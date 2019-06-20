/*
 * web-based control and visualisation
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <poll.h>

#include <linux/videodev2.h>

#include <jpeglib.h>
#include <libv4l2.h>

#include "sandan-v4l.h"

//int framerate = 25;
//int deliveredFrames = 0;

int v4l2_compress_jpeg(int width, int height, unsigned char *src, int src_size,
        unsigned char *dest, int dest_size) {
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    JSAMPROW row_pointer[1];
    unsigned char *buf = dest;
    unsigned long buf_size = dest_size;
    int row_stride;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    cinfo.image_width = width;
    cinfo.image_height = height;
    cinfo.input_components = 3;

    cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, 85, FALSE);

    jpeg_mem_dest(&cinfo, &buf, &buf_size);

    jpeg_start_compress(&cinfo, TRUE);

    row_stride = cinfo.image_width * cinfo.input_components;

    while (cinfo.next_scanline < cinfo.image_height) {
        row_pointer[0] = &src[cinfo.next_scanline * row_stride];
        jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);

    jpeg_destroy_compress(&cinfo);

    if (buf != dest) {
        printf("Destination memory to small (dest: %px%d, new: %px%ld)\n", dest,
                dest_size, buf, buf_size);
        return 0;
    }

    return buf_size;
}
int sandan_v4l_init(sandan_v4l *v4l, const char *device, int width, int height,
        int framerate) {
    memset(v4l, 0, sizeof(*v4l));

    v4l->device = strdup(device);
    v4l->width = width;
    v4l->height = height;
    v4l->framerate = framerate;

    v4l->fd = -1;

    return 1;
}

int sandan_v4l_start(sandan_v4l *v4l) {
    struct v4l2_format fmt;
    struct v4l2_buffer buf;
    struct v4l2_requestbuffers req;
    enum v4l2_buf_type type;
    int i;

    v4l->fd = v4l2_open(v4l->device, O_RDWR | O_NONBLOCK, 0);
    //v4l->fd = v4l2_open(v4l->device, O_RDWR);

    if (v4l->fd == -1) {
        printf("Couldn't open capture device %s\n", v4l->device);
        return -1;
    }

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = v4l->width;
    fmt.fmt.pix.height = v4l->height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;

    if (v4l2_ioctl(v4l->fd, VIDIOC_S_FMT, &fmt) == -1) {
        printf("Unsupported VIDIOC_S_FMT\n");
        return -1;
    }

    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB24) {
        printf("Pixelformat (%c%c%c%c) not supported\n",
                (char) fmt.fmt.pix.pixelformat,
                (char) (fmt.fmt.pix.pixelformat >> 8),
                (char) (fmt.fmt.pix.pixelformat >> 16),
                (char) (fmt.fmt.pix.pixelformat >> 24));
        return -1;
    }

    v4l->width = fmt.fmt.pix.width;
    v4l->height = fmt.fmt.pix.height;

    memset(&req, 0, sizeof(req));
    req.count = 2;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (v4l2_ioctl(v4l->fd, VIDIOC_REQBUFS, &req) == -1) {
        printf("Unsupported VIDIOC_REQBUFS\n");
        return -1;
    }

    v4l->buffers = calloc(req.count, sizeof(*v4l->buffers));
    for (i = 0; i < req.count; ++i) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (v4l2_ioctl(v4l->fd, VIDIOC_QUERYBUF, &buf) == -1) {
            printf("Unsupported VIDIOC_QUERYBUF\n");
            return -1;
        }

        v4l->buffers[i].length = buf.length;
        v4l->buffers[i].start = v4l2_mmap(NULL, buf.length,
                PROT_READ | PROT_WRITE, MAP_SHARED, v4l->fd, buf.m.offset);

        if (v4l->buffers[i].start == MAP_FAILED ) {
            printf("Mmap failed\n");
            return -1;
        }
    }
    v4l->buffers_count = i;

    for (i = 0; i < v4l->buffers_count; ++i) {
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (v4l2_ioctl(v4l->fd, VIDIOC_QBUF, &buf) == -1) {
            printf("Unsupported VIDIOC_QBUF\n");
            return -1;
        }
    }

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (v4l2_ioctl(v4l->fd, VIDIOC_STREAMON, &type) == -1) {
        printf("Unsupported VIDIOC_STREAMON\n");
        return -1;
    }

    return 1;
}

int sandan_v4l_read(sandan_v4l *v4l, unsigned char *image, int image_size) {

    struct v4l2_buffer buf;
    int n = -1;

    if (v4l->fd == -1)
        return -1;

    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (v4l2_ioctl(v4l->fd, VIDIOC_DQBUF, &buf) == -1) {
        printf("Unsupported VIDIOC_DQBUF\n");
        return 0;
    }

//    if (deliveredFrames < v4l->framerate) {
//        n = v4l2_compress_jpeg(v4l->width, v4l->height,
//                v4l->buffers[buf.index].start, buf.bytesused, image,
//                image_size);
//        deliveredFrames++;
//    } else {
//        deliveredFrames++;
//        if (deliveredFrames >= framerate){
//            deliveredFrames = 0;
//        }
//    }

    n = v4l2_compress_jpeg(v4l->width, v4l->height,
                    v4l->buffers[buf.index].start, buf.bytesused, image,
                    image_size);

    if (v4l2_ioctl(v4l->fd, VIDIOC_QBUF, &buf) == -1) {
        printf("Unsupported VIDIOC_QBUF\n");
        return -1;
    }
    return n;

}

int sandan_v4l_read_poll(sandan_v4l *v4l, unsigned char *image, int image_size,
        int timeout) {

    if (v4l->fd == -1) {
        return -1;
    }

    struct pollfd v4lpollfd;

    v4lpollfd.fd = (int) (long) v4l->fd;
    v4lpollfd.events = POLLOUT | POLLRDNORM;
    v4lpollfd.revents = 0;

    if (poll(&v4lpollfd, 1, timeout) > 0) {

        struct v4l2_buffer buf;
        int n;

        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (v4l2_ioctl(v4l->fd, VIDIOC_DQBUF, &buf) == -1) {
            printf("Unsupported VIDIOC_DQBUF\n");
            return 0;
        }
        n = v4l2_compress_jpeg(v4l->width, v4l->height,
                v4l->buffers[buf.index].start, buf.bytesused, image,
                image_size);

        if (v4l2_ioctl(v4l->fd, VIDIOC_QBUF, &buf) == -1) {
            printf("Unsupported VIDIOC_QBUF\n");
            return -1;
        }

        return n;
    } else {
        return -1;
    }
}

int sandan_v4l_stop(sandan_v4l *v4l) {
    enum v4l2_buf_type type;
    int n;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (v4l2_ioctl(v4l->fd, VIDIOC_STREAMOFF, &type) == -1) {
        printf("Unsupported VIDIOC_STREAMOFF\n");
        return -1;
    }

    for (n = 0; n < v4l->buffers_count; ++n)
        v4l2_munmap(v4l->buffers[n].start, v4l->buffers[n].length);
    free(v4l->buffers);
    v4l->buffers_count = 0;

    v4l2_close(v4l->fd);
    v4l->fd = -1;

    return 1;
}

void sandan_v4l_destroy(sandan_v4l *v4l) {
    if (v4l->fd != -1)
        sandan_v4l_stop(v4l);

    free(v4l->device);
    v4l->device = NULL;
}
