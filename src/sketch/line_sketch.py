import argparse
import cv2
import numpy as np


def resize_keep_ratio(image, max_size: int):
    h, w = image.shape[:2]
    if max(h, w) <= max_size:
        return image
    s = max_size / float(max(h, w))
    return cv2.resize(image, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA)


def thin_edges(bin_img_0_255):
    # opencv-contrib 필요: pip install opencv-contrib-python
    if hasattr(cv2, "ximgproc") and hasattr(cv2.ximgproc, "thinning"):
        return cv2.ximgproc.thinning(bin_img_0_255)
    return bin_img_0_255


def remove_small_components(bin_img, min_area):
    if min_area <= 0:
        return bin_img
    num, labels, stats, _ = cv2.connectedComponentsWithStats((bin_img > 0).astype(np.uint8), 8)
    out = np.zeros_like(bin_img)
    for i in range(1, num):
        if stats[i, cv2.CC_STAT_AREA] >= min_area:
            out[labels == i] = 255
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("image")
    ap.add_argument("-o", "--out", default="out_line.png")
    ap.add_argument("--max-size", type=int, default=1600)

    # 이진화 방식
    ap.add_argument("--adaptive", action="store_true", help="use adaptive threshold (for uneven lighting)")
    ap.add_argument("--block", type=int, default=31, help="adaptive block size (odd)")
    ap.add_argument("--C", type=int, default=7, help="adaptive C")

    # 끊김 보정 (사인에서 과하면 뭉개짐)
    ap.add_argument("--close-k", type=int, default=3, help="closing kernel size (odd), 0 to disable")
    ap.add_argument("--close-iter", type=int, default=1)

    # 노이즈 제거
    ap.add_argument("--min-area", type=int, default=30)

    # 최종 선 두께(보기 좋게)
    ap.add_argument("--thicken", type=int, default=0, help="dilate skeleton by N (0=off)")

    ap.add_argument("--debug", action="store_true")
    args = ap.parse_args()

    img0 = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if img0 is None:
        raise FileNotFoundError(f"Cannot read: {args.image}")

    img = resize_keep_ratio(img0, args.max_size)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 1) 살짝 블러(점노이즈만 정리)
    gray_blur = cv2.GaussianBlur(gray, (3, 3), 0)

    # 2) 이진화: 사인은 OTSU가 대부분 제일 깔끔
    if args.adaptive:
        b = args.block
        if b % 2 == 0:
            b += 1
        bin_img = cv2.adaptiveThreshold(
            gray_blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, b, args.C
        )
    else:
        _, bin_img = cv2.threshold(gray_blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    # 3) 끊김 보정(아주 약하게)
    if args.close_k and args.close_k > 0:
        k = args.close_k
        if k % 2 == 0:
            k += 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        it = max(1, min(args.close_iter, 2))
        bin_img = cv2.morphologyEx(bin_img, cv2.MORPH_CLOSE, kernel, iterations=it)

    # 4) 작은 조각 제거
    bin_img = remove_small_components(bin_img, args.min_area)

    # 5) 스켈레톤(1px)
    skel = thin_edges(bin_img)

    # 6) 보기 좋게 약간 두껍게(선택)
    if args.thicken > 0:
        kk = 2 * args.thicken + 1
        ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kk, kk))
        skel = cv2.dilate(skel, ker, iterations=1)

    # 7) 흰 배경 + 검은 선
    out = 255 - skel
    cv2.imwrite(args.out, out)
    print(f"[OK] saved: {args.out}")

    if args.debug:
        # cv2.imwrite("dbg_1_gray.png", gray)
        cv2.imwrite("dbg_2_bin.png", bin_img)
        # cv2.imwrite("dbg_3_skel.png", skel)
        print("[DBG] saved dbg_1_gray.png dbg_2_bin.png dbg_3_skel.png")


if __name__ == "__main__":
    main()
