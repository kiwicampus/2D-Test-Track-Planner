#!/usr/bin/env python3
# =============================================================================
import os
import cv2
import inspect


class bcolors:
    """!
    Class for defining the color used by the printlog function
    """

    LOG = {
        "WARN": ["\033[93m", "WARN"],
        "ERROR": ["\033[91m", "ERROR"],
        "OKGREEN": ["\033[32m", "INFO"],
        "OKPURPLE": ["\033[35m", "INFO"],
        "INFO": ["\033[0m", "INFO"],  # ['\033[94m', "INFO"],
        "BOLD": ["\033[1m", "INFO"],
        "GRAY": ["\033[90m", "INFO"],
    }
    BOLD = "\033[1m"
    ENDC = "\033[0m"
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    GRAY = "\033[90m"
    UNDERLINE = "\033[4m"


def printlog(msg: str, msg_type: str = "INFO", flush: bool = True):

    """!
    Object class constructor. Node to postprocess and filter segmentation using stereocamera information.
    @param msg `string` message to print
    @param msg_type string` message type
    @param flush  `boolean` sure that any output is buffered and go to the destination.
    """

    if not flush:
        return

    org = os.path.splitext(os.path.basename(inspect.stack()[1][1]))[0].upper()
    caller = inspect.stack()[1][3].upper()
    _str = "[{}][{}][{}]: {}".format(bcolors.LOG[msg_type][1], org, caller, msg)

    print(bcolors.LOG[msg_type][0] + _str + bcolors.ENDC, flush=True)


def overlay_image(l_img, s_img, pos, transparency, src_center=False):
    """Overlay 's_img on' top of 'l_img' at the position specified by
        pos and blend using 'alpha_mask' and 'transparency'.
    Args:
        l_img: `cv2.mat` inferior image to overlay superior image
        s_img: `cv2.mat` superior image to overlay
        pos: `tuple`  position to overlay superior image [pix, pix]
        transparency: `float` transparency in overlayed image
        src_center: `boolean` pos coordinate is the center of the image
    Returns:
        l_img: `cv2.mat` original image with s_img overlayed
    """

    # Center image
    if src_center:
        pos = list(pos)
        pos[0] -= int(s_img.shape[1] * 0.5)
        pos[1] -= int(s_img.shape[0] * 0.5)

    # Get superior image dimensions
    s_img_height, s_img_width, s_img_channels = s_img.shape

    if s_img_channels == 3 and transparency != 1:
        s_img = cv2.cvtColor(s_img, cv2.COLOR_BGR2BGRA)
        s_img_channels = 4

    # Take 3rd channel of 'img_overlay' image to get shapes
    img_overlay = s_img[:, :, 0:4]

    # cords assignation to overlay image
    x, y = pos

    # Image ranges
    y1, y2 = max(0, y), min(l_img.shape[0], y + img_overlay.shape[0])
    x1, x2 = max(0, x), min(l_img.shape[1], x + img_overlay.shape[1])

    # Overlay ranges
    y1o, y2o = max(0, -y), min(img_overlay.shape[0], l_img.shape[0] - y)
    x1o, x2o = max(0, -x), min(img_overlay.shape[1], l_img.shape[1] - x)

    # Exit if nothing to do
    if y1 >= y2 or x1 >= x2 or y1o >= y2o or x1o >= x2o:
        return l_img

    if s_img_channels == 4:
        # Get alphas channel
        alpha_mask = (s_img[:, :, 3] / 255.0) * transparency
        alpha_s = alpha_mask[y1o:y2o, x1o:x2o]
        alpha_l = 1.0 - alpha_s

        # Do the overlay with alpha channel
        for c in range(0, l_img.shape[2]):
            l_img[y1:y2, x1:x2, c] = (
                alpha_s * img_overlay[y1o:y2o, x1o:x2o, c]
                + alpha_l * l_img[y1:y2, x1:x2, c]
            )

    elif s_img_channels < 4:
        # Do the overlay with no alpha channel
        if l_img.shape[2] == s_img.shape[2]:
            l_img[y1:y2, x1:x2] = s_img[y1o:y2o, x1o:x2o]
        else:
            print("Error: to overlay images should have the same color channels")
            return l_img

    # Return results
    return l_img


def print_list_text(
    img_src,
    str_list,
    origin=(0, 0),
    color=(0, 255, 255),
    line_break=20,
    thickness=2,
    fontScale=0.45,
):
    """
        Draws a list with strings
    Args:
        img_src: `cv2.math` image to draw text
        str_list: `list` with string to print
        origin: `tuple` origin to start text (X, Y)
        color: `tuple` color (B, G, R)
        line_break: `int` space between new text lines
        thickness: `int` text thickness
        fontScale: `float` text font scale
    Returns:
        img_src: `type` description
    """

    for idx, strprint in enumerate(str_list):
        cv2.putText(
            img=img_src,
            text=strprint,
            org=(origin[0], origin[1] + (line_break * idx)),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=fontScale,
            color=(0, 0, 0),
            thickness=thickness + 3,
        )
        cv2.putText(
            img=img_src,
            text=strprint,
            org=(origin[0], origin[1] + (line_break * idx)),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=fontScale,
            color=color,
            thickness=thickness,
        )

    return img_src
