# How to set up custom payload

The following is a set of guidelines for setting up the payload proposed in the paper [Lowering Barriers to Entry for Fully-Integrated Custom Payloads on a DJI Matrice](https://arxiv.org/abs/2405.06176): https://arxiv.org/abs/2405.06176.

## Overview

We propose a top-mounted, generic computational payload intended to be cheap and customizable. It uses the E-port for accessing the drone's camera feeds, sensors and data topics, and for controlling the drone's flight and the behavior of other payloads. It uses the PSDK port for streaming video to the controller using the drone's transmission infrastructure, so that an operator can see a custom visualization in real time without adding transmission equipment to the drone. Currently, this requires two PSDK applications to be running simultaneously -- one for each port. DJI may of course change this in the future. However, since the applications are not interconnected, one such application can be run on its own if the functionality of one of the ports is not required. This demo runs the DJI E-port demo code (accesses camera feeds, sensor data, controls flight and other payloads), and a custom application to stream the desktop of the Raspberry Pi to the controller. Finally, the payload fits in a 3D-printed case with quick-release brackets for convenient and quick installation in the field.

|  Installs on top  |  Fits in M350 case  |
| ------------- | ------------- |
| ![matrice_landed](https://github.com/uzgit/Payload-SDK/assets/14451567/0591db04-fbdc-40a4-b78b-ef8c1c4b73fe)  |  ![payload_in_case](https://github.com/uzgit/Payload-SDK/assets/14451567/ae6e5fb7-1634-4dd9-acae-dd766647ca67) |
| **Interior** | **Streams to Controller** |
| ![payload_electronics](https://github.com/uzgit/Payload-SDK/assets/14451567/48fe10d7-0b4d-4516-a579-6add097f0512)  |  ![payload_desktop_stream](https://github.com/uzgit/Payload-SDK/assets/14451567/8cc97706-ff47-4ac2-aa09-a77d23e5d6d3) |

## Physical setup

For creating the 3D-printed parts, see the Thingiverse entry here: https://www.thingiverse.com/thing:6681945.

We are using the following electronics:
* **Raspberry PI 5:** the whole reason for the existence of this payload -- a Linux computational environment that provides access to the drone.
* **E-port expansion board:** an interface providing USB-C access to the drone via its E-port (https://store.dji.com/product/dji-e-port-development-kit?vid=141171)
* **SkyPort V2 expansion board:** an interface providing ethernet access to the drone via its PSDK port -- and implicitly via an upward gimbal connector. (https://store.dji.com/product/psdk-development-kit-v2?vid=89481, https://store.dji.com/product/m300-upward-gimbal-connector?vid=100911)
* **DFR0753 DC-to-DC converter:** a power supply unit that can provide a more reliable 5V source to the Raspberry Pi than the expansion boards can.

### Power
The Raspberry Pi 5 can consume 5A at 5V, which is a lot to keep up with. Although the E-port board provides a 5V source, it is unable to power the Pi at full speed according to our tests. Therefore, we instead pull from the E-port's 12V source, stepping down to 5V with the DFR0753, which is more reliable. We connect the output of the DFR0753 to the Raspberry Pi's 5V and GND GPIO pins. This is the only connection that is explicitly for power on this setup.

### Data
In general, the PSDK expansion boards require data connections each
* UART for lower-level configuration, handshaking, and low-bandwidth data transfer
* network or USB bulk for high-bandwidth data transfer such as video

We connect the UART port on the E-port expansion board to the UART port on the Raspberry Pi's GPIO pins. We connect the USB-C connector on the E-port board to the Raspberry Pi's main USB-C connector. This provides power and data to the Pi. We transfer data using 2 bulk devices configured over this single USB link (explained later). It is possible to use Ethernet over USB, but this interferes with the SkyPort V2 expansion board, which requires Ethernet. Further, fewer data sources are available over the Ethernet over USB link, e.g., the drone's onboard depth cameras.

We connect the UART port on the SkyPort V2 expansion board to a UART-to-USB converter which connects to the Raspberry Pi's USB-A port. We connect the Ethernet port on the SkyPort V2 expansion board to the Raspberry Pi's Ethernet port. The SkyPort V2 expansion board connects to the SkyPort V2 connector using its included cable, and the SkyPort V2 connector connects to an upward gimbal connector, which connects to the drone's PSDK port. We have disassembled the upward gimbal connector and the SkyPort V2 connector and reassembled them using a low-profile 3D printed assembler that allows the pieces to fit inside the payload case. We do not draw power from the SkyPort V2 expansion board.

## Computational setup

This part is non-trivial and its difficulty should not be underestimated. We will lay out the key steps of our setup, which implemented in more detail in both our [our configuration script](https://raw.githubusercontent.com/uzgit/Payload-SDK/master/configure.sh) and our code repository, and which may or may not generalize to other platforms, OS versions, PSDK versions, etc. We have only tested this for the Raspberry Pi 5, and any deviations from our setup will require their own configuration. The following is intended for the 32-bit Raspbian Bookworm image from 2024-03-15 and on PSDK v3.8.

We use the Wayland desktop environment because it makes it easier to stream the Raspberry Pi's desktop using ```ffmpeg```, which DJI video encoding depends on. We set the resolution low in order to keep the computational load reduced.
```
echo "Switching from Wayland to X11..." 
sudo raspi-config nonint do_wayland W1

echo "Setting VNC resolution to 640x480..."
sudo raspi-config nonint do_vnc_resolution 640x480
```

We install a bunch of dependencies...
```
echo "Installing basic required software and development packages"
sudo apt-get update
sudo apt-get -y install \
    autoconf \
    automake \
    bridge-utils \
    build-essential \
    cmake \
    dnsmasq \
    doxygen \
    git \
    graphviz \
    htop \
    imagemagick \
    libaio-dev \
    libaom-dev \
    libaom3 \
    libaom-doc \
    libasound2-dev \
    libass-dev \
    libavcodec-dev \
    libavdevice-dev \
    libavfilter-dev \
    libavformat-dev \
    libavutil-dev \
    libfreetype6-dev \
    libgmp-dev \
    libmp3lame-dev \
    libopencore-amrnb-dev \
    libopencore-amrwb-dev \
    libopencv-dev \
    libopus-dev \
    libopus0 \
    librtmp-dev \
    libsdl2-dev \
    libsdl2-image-dev \
    libsdl2-mixer-dev \
    libsdl2-net-dev \
    libsdl2-ttf-dev \
    libsnappy-dev \
    libsoxr-dev \
    libssh-dev \
    libssl-dev \
    libtool \
    libusb-1.0-0-dev \
    libv4l-dev \
    libva-dev \
    libvdpau-dev \
    libvo-amrwbenc-dev \
    libvorbis-dev \
    libwebp-dev \
    libx11-dev \
    libx264-dev \
    libx265-dev \
    libxcb-shape0-dev \
    libxcb-shm0-dev \
    libxcb-xfixes0-dev \
    libxcb1-dev \
    libxml2-dev \
    libzimg2 \
    libzimg-dev \
    lzma-dev \
    meson \
    mlocate \
    nasm \
    openssh-server \
    pkg-config \
    python3-dev \
    python3-pip \
    texinfo \
    tree \
    wget \
    vim \
    xdotool \
    yasm \
    zlib1g-dev
```

We put ```libopus.a``` on the path so it can be found during the PSDK compilation:
```
# put opus on the path so that the PSDK can find it (32-bit Raspbian needs this)
sudo ln -s /usr/lib/arm-linux-gnueabihf/libopus.a /usr/local/lib/libopus.a
```

We enable the DWC2 module in ```/boot/firmware/config.txt```, which helps for using the Raspberry Pi as an OTG gadget and generally makes it easier to transfer data over the main USB-C connector:
```
echo "Making edits to cmdline.txt (load dwc2)"
filename=/boot/firmware/cmdline.txt
text='modules-load=dwc2'
file=$(realpath $filename)
echo "File path: $file"
echo "Adding parameters to $file"
# Check if the text already exists in the file
if ! grep -qF "$text" "$file"; then
        # If not, append the text to the end of the file
        sudo bash -c "sed -i '{s/$/ $text/}' $file" \
                && echo "$text appended successfully to $file" \
                || echo "FAILED TO ADD $text TO FILE!"
else
        # If the text already exists, display a message
        echo "$text already exists in $file."
fi
echo
```

We enable ```uart0``` in ```/boot/firmware/config.txt``` (We also do the same for ```uart1```):
```
echo "Making edits to config.txt (enable uart0)"
filename=/boot/firmware/config.txt
text='dtoverlay=uart0'
file=$(realpath $filename)
echo "File path: $file"
echo "Adding parameters to $file"
# Check if the text already exists in the file
if ! grep -qF "$text" "$file"; then
        # If not, append the text to the end of the file
        sudo bash -c "echo '$text' >> $file"
        echo "$text appended successfully to $file"
else
        # If the text already exists, display a message
        echo "$text already exists in $file."
fi
echo
```

We enable ```libcomposite``` in ```/etc/modules``` which helps with using the Raspberry Pi as a composite USB gadget, thereby making the USB bulk devices possible.
```
echo "Making edits to config.txt (enable libcomposite)"
filename=/etc/modules
text='libcomposite'
file=$(realpath $filename)
echo "File path: $file"
echo "Adding parameters to $file"
# Check if the text already exists in the file
if ! grep -qF "$text" "$file"; then
        # If not, append the text to the end of the file
        sudo bash -c "echo '$text' >> $file"
        echo "$text appended successfully to $file"
else
        # If the text already exists, display a message
        echo "$text already exists in $file."
fi
echo
```

We create a setup script adapted from [this tutorial](https://www.hardill.me.uk/wordpress/2023/12/23/pi5-usb-c-gadget/), which runs on boot, activates composite USB device functions, sets the Raspberry Pi's USB identifying information, and activates USB bulk devices:
```
#!/bin/bash 

cd /sys/kernel/config/usb_gadget/ 
mkdir -p pi 

cd pi 
#echo 0x1d6b > idVendor # Linux Foundation 
#echo 0x0104 > idProduct # Multifunction Composite Gadget 
echo 0x0955 > idVendor # Linux Foundation 
echo 0x7020 > idProduct # Multifunction Composite Gadget 
echo 0x0001 > bcdDevice # v1.0.0 

echo 0x0200 > bcdUSB # USB2 

echo 0xEF > bDeviceClass 
echo 0x02 > bDeviceSubClass 
echo 0x01 > bDeviceProtocol 

mkdir -p strings/0x409 
echo "abcdefg1234567890" > strings/0x409/serialnumber 
echo "raspberry" > strings/0x409/manufacturer 
echo "PI5" > strings/0x409/product 

cfg=configs/c.1 
mkdir -p "${cfg}" 
echo 0x80 > ${cfg}/bmAttributes 
echo 250 > ${cfg}/MaxPower 

cfg_str="" 
udc_dev=fe980000.usb 

# The IP address and associated netmask shared by all USB network interfaces created by this script. 
net_ip=192.168.55.1 
net_mask=255.255.255.0 

# Note: RNDIS must be the first function in the configuration, or Windows 
# RNDIS support will not operate correctly. 
enable_rndis=1 
if [ ${enable_rndis} -eq 1 ]; then 
    cfg_str="${cfg_str}+RNDIS" 
    func=functions/rndis.usb0 
    mkdir -p "${func}" 
    ln -sf "${func}" "${cfg}" 

    # Informs Windows that this device is compatible with the built-in RNDIS 
    # driver. This allows automatic driver installation without any need for 
    # a .inf file or manual driver selection. 
    echo 1 > os_desc/use 
    echo 0xcd > os_desc/b_vendor_code 
    echo MSFT100 > os_desc/qw_sign 
    echo RNDIS > "${func}/os_desc/interface.rndis/compatible_id" 
    echo 5162001 > "${func}/os_desc/interface.rndis/sub_compatible_id" 
    ln -sf "${cfg}" os_desc 
fi 

systemctl daemon-reload

enable_bulk=1 
if [ ${enable_bulk} -eq 1 ]; then 
    mkdir -p /dev/usb-ffs 
    mkdir -p /dev/usb-ffs/bulk1
    mkdir -p /dev/usb-ffs/bulk2
    
    cfg_str="${cfg_str}+BULK1" 
    func=functions/ffs.bulk1
    mkdir -p "${func}" 
    ln -sf "${func}" configs/c.1/ 

    cfg_str="${cfg_str}+BULK2" 
    func=functions/ffs.bulk2
    mkdir -p "${func}" 
    ln -sf "${func}" configs/c.1/ 
      
    mount -o mode=0777 -t functionfs bulk1 /dev/usb-ffs/bulk1
    mount -o mode=0777 -t functionfs bulk2 /dev/usb-ffs/bulk2
EOF1-start-dji-usb-gadget.sh
echo "(exec ${HOME}/startup-bulk /dev/usb-ffs/bulk1 &)" >> ~/start-dji-usb-gadget.sh
echo "(exec ${HOME}/startup-bulk /dev/usb-ffs/bulk2 &)" >> ~/start-dji-usb-gadget.sh
cat >> ~/start-dji-usb-gadget.sh << 'EOF2-start-dji-usb-gadget.sh'

    sleep 3
    
    mkdir -p "${cfg}/strings/0x409" 
    echo "${cfg_str:1} " > "${cfg}/strings/0x409/configuration" 
fi

udevadm settle -t 5 || :
ls /sys/class/udc > UDC 

/sbin/brctl addbr br0 
/sbin/ifconfig br0 ${net_ip} netmask ${net_mask} up 

if [ ${enable_rndis} -eq 1 ]; then 
    /sbin/brctl addif br0 usb0 
    /sbin/ifconfig usb0 down 
    /sbin/ifconfig usb0 up 
fi 

echo "Finished USB bulk device creation!"

exit 0
```

Part of the USB bulk device initialization is to run a program provided by [DJI's Chinese language tutorial for the M30 and Raspberry Pi 4](https://sdk-forum.dji.net/hc/zh-cn/articles/10232604141465-%E6%A0%91%E8%8E%93%E6%B4%BE4B%E9%85%8D%E7%BD%AEUSB-device-RNDIS-%E5%92%8C-BULK). We modify the program to set up 2 USB bulk devices, because the M350 uses 2 of them, and so we use the ```.c``` source below:
```
/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 - 2016 Samsung Electronics
 *	   Krzysztof Opasiak <k.opasiak@samsung.com>
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT
 * OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <malloc.h>
#include <stdlib.h>
#include <unistd.h>
#include <poll.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/eventfd.h>


#include <libaio.h>
#include <linux/usb/functionfs.h>

#define EP_IN_IDX 1
#define EP_OUT_IDX 2

#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define container_of(ptr, type, member) ({                      \
        const typeof( ((type *)0)->member ) *__mptr = (ptr);    \
        (type *)( (char *)__mptr - offsetof(type,member) );})
/************************ Little Endian Handling **************************** ******/ 

/* 
* cpu_to_le16/32 are used when initializing structures, a context where a 
* function call is not allowed. To solve this, we code cpu_to_le16/32 in a way 
* that allows them to be used when initializing structures. 
*/ 

#if __BYTE_ORDER == __LITTLE_ENDIAN 
#define cpu_to_le16(x) (x) 
#define cpu_to_le32(x) (x) 
#else 
#define cpu_to_le16(x) ((((x) >> 8) & 0xffu) | (((x) & 0xffu) << 8)) 
#define cpu_to_le32(x) \ 
((((x) & 0xff000000u) >> 24) | (((x) & 0x00ff0000u) >> 8) | \ 
( ((x) & 0x0000ff00u) << 8) | (((x) & 0x000000ffu) << 24)) 
#endif 

#define le32_to_cpu(x) le32toh(x) 
#define le16_to_cpu(x) le16toh(x)
/******************** Descriptors and Strings *******************************/

static const struct {
	struct usb_functionfs_descs_head_v2 header;
	__le32 fs_count;
	__le32 hs_count;
	struct {
		struct usb_interface_descriptor intf;
		struct usb_endpoint_descriptor_no_audio bulk_in;
		struct usb_endpoint_descriptor_no_audio bulk_out;
	} __attribute__ ((__packed__)) fs_descs, hs_descs;
} __attribute__ ((__packed__)) descriptors = {
	.header = {
		.magic = cpu_to_le32(FUNCTIONFS_DESCRIPTORS_MAGIC_V2),
		.flags = cpu_to_le32(FUNCTIONFS_HAS_FS_DESC |
				     FUNCTIONFS_HAS_HS_DESC),
		.length = cpu_to_le32(sizeof(descriptors)),
	},
	.fs_count = cpu_to_le32(3),
	.fs_descs = {
		.intf = {
			.bLength = sizeof(descriptors.fs_descs.intf),
			.bDescriptorType = USB_DT_INTERFACE,
			.bNumEndpoints = 2,
			.bInterfaceClass = USB_CLASS_VENDOR_SPEC,
			.iInterface = 1,
		},
		.bulk_in = {
			.bLength = sizeof(descriptors.fs_descs.bulk_in),
			.bDescriptorType = USB_DT_ENDPOINT,
			.bEndpointAddress = 1 | USB_DIR_IN,
			.bmAttributes = USB_ENDPOINT_XFER_BULK,
		},
		.bulk_out = {
			.bLength = sizeof(descriptors.fs_descs.bulk_out),
			.bDescriptorType = USB_DT_ENDPOINT,
			.bEndpointAddress = 2 | USB_DIR_OUT,
			.bmAttributes = USB_ENDPOINT_XFER_BULK,
		},
	},
	.hs_count = cpu_to_le32(3),
	.hs_descs = {
		.intf = {
			.bLength = sizeof(descriptors.hs_descs.intf),
			.bDescriptorType = USB_DT_INTERFACE,
			.bNumEndpoints = 2,
			.bInterfaceClass = USB_CLASS_VENDOR_SPEC,
			.iInterface = 1,
		},
		.bulk_in = {
			.bLength = sizeof(descriptors.hs_descs.bulk_in),
			.bDescriptorType = USB_DT_ENDPOINT,
			.bEndpointAddress = 1 | USB_DIR_IN,
			.bmAttributes = USB_ENDPOINT_XFER_BULK,
			.wMaxPacketSize = cpu_to_le16(512),
		},
		.bulk_out = {
			.bLength = sizeof(descriptors.hs_descs.bulk_out),
			.bDescriptorType = USB_DT_ENDPOINT,
			.bEndpointAddress = 2 | USB_DIR_OUT,
			.bmAttributes = USB_ENDPOINT_XFER_BULK,
			.wMaxPacketSize = cpu_to_le16(512),
		},
	},
};

/*
 * In previous workshops we used Loopback function
 * which has exactly this string. To make this
 * workshop compatible with all previous workshops
 * we declare the same string as Loopback function
 */
#define STR_INTERFACE "loop input to output"

static const struct {
	struct usb_functionfs_strings_head header;
	struct {
		__le16 code;
		const char str1[sizeof(STR_INTERFACE)];
	} __attribute__ ((__packed__)) lang0;
} __attribute__ ((__packed__)) strings = {
	.header = {
		.magic = cpu_to_le32(FUNCTIONFS_STRINGS_MAGIC),
		.length = cpu_to_le32(sizeof(strings)),
		.str_count = cpu_to_le32(1),
		.lang_count = cpu_to_le32(1),
	},
	.lang0 = {
		cpu_to_le16(0x0409), /* en-us */
		STR_INTERFACE,
	},
};

#define EXIT_COMMAND "\\exit"
#define MAX_LINE_LENGTH 1024*8 /* 8 KB should be enough for single line */

#define report_error(...) do {			\
		fprintf(stderr, __VA_ARGS__);	\
		putchar('\n');			\
	} while (0)

/* Data structure for our message */
struct message {
	uint16_t length;
	char line_buf[MAX_LINE_LENGTH];
} __attribute((packed));

/*
 * As there is no fully-functional equivalent of libusb for the device
 * side we define some helper structures to make our life easier and
 * use them as some minimal equivalent of such library
 */
struct ffs_request;

typedef void (*ffs_complete_t)(struct ffs_request *);

/* Possible states of ffs request */
enum {
	FFS_REQ_COMPLETED = 0,
	FFS_REQ_IN_PROGRESS = 1,
	FFS_REQ_ERROR = 2,
	FFS_REQ_CANCELLED = 3,
};

/* Represents single usb request which can be transfered using ffs */
struct ffs_request {
	struct iocb iocb;
	unsigned char *buf;
	ssize_t length;
	void *context;
	int status;
	int actual;
	ffs_complete_t complete;
};

/* Use container_of() to get ffs_request from iocb */
static inline struct ffs_request *to_ffs_request(struct iocb *_iocb)
{
	return container_of(_iocb, struct ffs_request, iocb);
}

/*
 * helper structure which represents transfer
 * of single message in our chat protocol
 */
struct transfer {
	struct ffs_request *length_request;
	struct ffs_request *buf_request;
	struct message message;
	int in_progress;
	io_context_t *ctx;
};

/* Indicates if device prompt is currently present */
int device_prompt;

/******************** Basic implementation of our library *********************/

/* allocates single ffs_request */
struct ffs_request *alloc_ffs_request()
{
	struct ffs_request *req;

	req = malloc(sizeof(*req));
	if (!req)
		goto out;

	memset(req, 0, sizeof(*req));
out:
	return req;
}

void free_ffs_request(struct ffs_request *req)
{
	free(req);
}

/*
 * Schedules ffs request to be asynchronously transfered.
 * A little bit device side equivalent to libusb_submit_transfer()
 */
int submit_ffs_request(io_context_t *ctx, struct ffs_request *req)
{
	int ret;
	struct iocb *iocb = &req->iocb;

	iocb->u.c.nbytes = req->length;

	ret = io_submit(*ctx, 1, &iocb);
	if (ret < 0)
		return ret;

	req->status = FFS_REQ_IN_PROGRESS;
	return 0;
}

/*
 * Fill given ffs request using provided data
 * A little bit device side equivalent of libusb_fill_bulk_transfer()
 */
void fill_ffs_request(struct ffs_request *req, int dir, int ep, int event_fd,
		      unsigned char *buf, int length, ffs_complete_t complete,
		      void *context)
{
	struct iocb *iocb = &req->iocb;

	req->buf = buf;
	req->length = length;

	/*
	 * TODO: prepare read/write operation
	 *
	 * Hints:
	 * - Use dir param to determine type of operation
	 * - Remember that we are on the device side
	 * - USB_DIR_IN - transfer data from device to host (write())
	 * - USB_DIR_OUT - transfer data from host to device (read())
	 *
	 * int io_prep_pwrite()
	 * int io_prep_pread()
	 */
	if (dir == USB_DIR_IN)
		io_prep_pwrite(iocb, ep, buf, length, 0);
	else
		io_prep_pread(iocb, ep, buf, length, 0);


	io_set_eventfd(iocb, event_fd);

	req->complete = complete;
	req->status = 0;
	req->actual = 0;
	req->context = context;
}

/* Cancel choosen ffs_request */
void cancel_ffs_request(io_context_t *ctx, struct ffs_request *req)
{
	struct io_event e;
	if (req->status != FFS_REQ_IN_PROGRESS)
		return;

	io_cancel(*ctx, &req->iocb, &e);
	req->status = FFS_REQ_CANCELLED;
	req->actual = 0;
	if (req->complete)
		req->complete(req);
}

/* Handle all pending aio events */
int handle_events(io_context_t *ctx, int event_fd)
{
	int ret;
	int i;
	uint64_t ev_cnt;
	struct io_event e[2];
	struct ffs_request *req;

	ret = read(event_fd, &ev_cnt, sizeof(ev_cnt));
	if (ret < 0) {
		report_error("unable to read eventfd");
		return -errno;
	}

	ret = io_getevents(*ctx, 1, ev_cnt, e, NULL);
	if (ret < 0)
		return ret;

	for (i = 0; i < ret; ++i) {
		long res = (long)e[i].res;

		req = to_ffs_request(e[i].obj);
		if (res >= 0)
			req->status = FFS_REQ_COMPLETED;
		else
			req->status = FFS_REQ_ERROR;

		req->actual = res;

		if (req->complete)
			req->complete(req);
	}

	return 0;
}

/* Prepare fresh ffs instance to communicate using our chat protocol */
int prepare_ffs(char *ffs_path, int *ep)
{
	char *ep_path;
	int i;
	int ret = 0;

	ep_path = malloc(strlen(ffs_path) + 4 /* "/ep#" */ + 1 /* '\0' */);
	if (!ep_path) {
		report_error("malloc");
		return -EINVAL;
	}

	/* open endpoint 0 */
	sprintf(ep_path, "%s/ep0", ffs_path);
	ep[0] = open(ep_path, O_RDWR);
	if (ep[0] < 0) {
		report_error("unable to open ep0");
		ret = -errno;
		goto out;
	}

	/*
	 * TODO: Provide descriptors and strings
	 *
	 * Hints:
	 * - Descriptors and strings are defined on the top of this file
	 * - You should simply two time call write() function using ep[0] fd
	 *
	 * ssize_t write(int fd, const void *buf, size_t count)
	 * sizeof()
	 */
	if (write(ep[0], &descriptors, sizeof(descriptors)) < 0) {
		report_error("unable do write descriptors");
		ret = -errno;
		goto out;
	}

	if (write(ep[0], &strings, sizeof(strings)) < 0) {
		report_error("unable to write strings");
		ret = -errno;
		goto out;
	}

out:
	free(ep_path);
	return ret;
}

/* Close all ep files */
void cleanup_ffs(int *ep)
{
	int i;

	for (i = 0; i < 3; ++i)
		close(ep[i]);
}

/* Initialize aio context and create event fd */
int init_aio(int n_requests, io_context_t *ctx, int *event_fd)
{
	int ret;

	memset(ctx, 0, sizeof(*ctx));

	ret = io_setup(n_requests, ctx);
	if (ret < 0) {
		report_error("Unable to setup aio context");
		return ret;
	}

	*event_fd = eventfd(0, 0);
	if (*event_fd < 0) {
		report_error("Unable to open event fd");
		io_destroy(*ctx);
		ret = -errno;
	}

	return ret;
}

/* Cleanup aio context and close event fd */
void cleanup_aio(io_context_t *ctx, int event_fd)
{
	close(event_fd);
	io_destroy(*ctx);
}

/******************** Protocol specific implementation **********************/

/* Alloc single message transfer (2 ffs requests) */
struct transfer *alloc_transfer()
{
	struct transfer *t;

	t = malloc(sizeof(*t));
	if (!t)
		goto out;

	t->length_request = alloc_ffs_request();
	if (!t->length_request)
		goto free_t;

	t->buf_request = alloc_ffs_request();
	if (!t->length_request)
		goto free_length_request;

	t->in_progress = 0;
	return t;

free_length_request:
	free_ffs_request(t->length_request);
free_t:
	free(t);
out:
	return NULL;
}

/* Free the memory allocated for requests */
static void free_transfer(struct transfer *t)
{
	if (!t)
		return;

	if (t->length_request)
		free_ffs_request(t->length_request);

	if (t->buf_request)
		free_ffs_request(t->buf_request);

	free(t);
}

/* Send chat message from device to host*/
int send_message(struct transfer *out_transfer)
{
	int len;
	int ret;

	len = strlen(out_transfer->message.line_buf) + 1;
	out_transfer->message.length = cpu_to_le16(len + 2);
	out_transfer->in_progress = 1;

	ret = submit_ffs_request(out_transfer->ctx,
				 out_transfer->length_request);
	if (ret)
		report_error("Unable send message");

	return ret;
}

/* Receive message from host to device */
int recv_message(struct transfer *in_transfer)
{
	int ret;

	in_transfer->in_progress = 1;

	ret = submit_ffs_request(in_transfer->ctx,
				 in_transfer->length_request);
	if (ret)
		report_error("Unable to receive message");

	return ret;
}

/* Called when message has been received  */
void in_complete(struct ffs_request *req)
{
	int ret;
	struct transfer *in_transfer = req->context;

	switch (req->status) {
	case FFS_REQ_COMPLETED:
		break;
	case FFS_REQ_CANCELLED:
		/* This means that we are closing our program */
		return;
	default:
		report_error("Failed to receive data");
		/* Just die to keep things simple */
		exit(-1);
	}

	if (in_transfer->length_request == req) {
		/*
		 * We have correctly received length of message
		 * lets wait for rest of data.
		 */
		int len;

		len = le16toh(in_transfer->message.length) - 2;

		/*
		 * TODO: Set the correct request length and submit it
		 *
		 * Hints:
		 * - In case of error, just exit(-1) to keep things simple
		 * - Use in_transfer->buf_request to receive message content
		 *
		 * struct ffs_request {
		 *  (...)
		 *  ssize_t length;
		 *  (...)
		 * };
		 *
		 * int submit_ffs_request()
		 */
		in_transfer->buf_request->length = len;

		ret = submit_ffs_request(in_transfer->ctx,
					 in_transfer->buf_request);
		if (ret < 0) {
			report_error("Failed to submit transfer");
			/* Just die to keep things simple */
			exit(-1);
		}
	} else {
		/*
		 * We have the whole message so lets print it
		 * and wait for another one
		 */
		in_transfer->in_progress = 0;
		//if (device_prompt)
		//	printf("<skip>\n");

		printf("recv data from host> %s\n", in_transfer->message.line_buf);

		//if (device_prompt)
		//	printf("device> ");

		fflush(stdout);

		ret = recv_message(in_transfer);
		if (ret < 0) {
			report_error("Failed to receive message");
			/* Just die to keep things simple */
			exit(-1);
		}

	}
}

/* Called when we successfully send any message to host */
void out_complete(struct ffs_request *req)
{
	int ret;
	struct transfer *out_transfer = req->context;

	switch (req->status) {
	case FFS_REQ_COMPLETED:
		break;
	case FFS_REQ_CANCELLED:
		/* This means that we are closing our program */
		return;
	default:
		report_error("Failed to send data");
		/* Just die to keep things simple */
		exit(-1);
	}

	if (out_transfer->length_request == req) {
		/*
		 * We have correctly send the length,
		 * so lets send now the data.
		 */
		int len;

		len = le16toh(out_transfer->message.length) - 2;
		/*
		 * TODO: Set the correct request length and submit it
		 *
		 * Hints:
		 * - In case of error, just exit(-1) to keep things simple
		 * - Use out_transfer->buf_request to send the data
		 *
		 * struct ffs_request {
		 *  (...)
		 *  ssize_t length;
		 *  (...)
		 * };
		 *
		 * int submit_ffs_request()
		 */
		out_transfer->buf_request->length = len;
		ret = submit_ffs_request(out_transfer->ctx,
					 out_transfer->buf_request);
		if (ret < 0) {
			report_error("Failed submit transfer");
			/* Just die to keep things simple */
			exit(-1);
		}
	} else {
		/*
		 * We have the whole message so lets print
		 * the prompt once again.
		 */
		out_transfer->in_progress = 0;
		device_prompt = 1;
		printf("device> ");
		fflush(stdout);
		/* Rest of the work will be done in do_chat() */
	}
}

/* prepare one in and one out chat transfers */
int prepare_transfers(int *ep, io_context_t *ctx, int event_fd,
		      struct transfer **in_transfer,
		      struct transfer **out_transfer)
{
	struct transfer *it, *ot;

	/* In our chat protocol we understand IN transfer
	 *  as receiving data, but on USB level thats are OUT
	 *  requests as data is being transfered from host to device
	 */
	it = alloc_transfer();
	if (!it)
		goto out;

	ot = alloc_transfer();
	if (!ot)
		goto free_it;

	/*
	 * TODO: Fill the USB OUT requests (for chat IN transfer)
	 *
	 * Hints:
	 * - We are on device side so we receives the data when
	 * USB request direction is OUT (from host to device)
	 * - use ep[EP_OUT_IDX] as endpoint file descriptor
	 * - use it->message as buffer for receiving data
	 * - use 2 as length for first request
	 * - no matter what you will use as length in second request
	 * as it will be overwritten in in_complete()
	 * - use in_complete() as completion callback
	 * - use it as user data as it will be needed later
	 *
	 * void fill_ffs_request()
	 */
	fill_ffs_request(it->length_request, USB_DIR_OUT, ep[EP_OUT_IDX],
			 event_fd, (unsigned char *)&it->message.length, 2,
			 in_complete, it);

	fill_ffs_request(it->buf_request, USB_DIR_OUT, ep[EP_OUT_IDX],
			 /*
			  * Actual length will be filled after
			  * receiving it from host
			  */
			 event_fd, (unsigned char *)&it->message.line_buf, 0,
			 in_complete, it);

	/*
	 * TODO: Fill the USB IN requests (for chat OUT transfer)
	 *
	 * Hints:
	 * - We are on device side so we send the data when
	 * USB request direction is IN (from device to host)
	 * - use ep[EP_IN_IDX] as endpoint file descriptor
	 * - use ot->message as buffer for receiving data
	 * - use 2 as length for first request
	 * - no matter what you will use as length in second request
	 * as it will be overwritten in out_complete()
	 * - use in_complete() as completion callback
	 * - use ot as user data as it will be needed later
	 *
	 * void fill_ffs_request()
	 */
	fill_ffs_request(ot->length_request, USB_DIR_IN, ep[EP_IN_IDX],
			 event_fd, (unsigned char *)&ot->message.length, 2,
			 out_complete, ot);

	fill_ffs_request(ot->buf_request, USB_DIR_IN, ep[EP_IN_IDX],
			 /*
			  * Actual length will be filled after
			  * reading user input
			  */
			 event_fd, (unsigned char *)&ot->message.line_buf, 0,
			 out_complete, ot);


	it->ctx = ctx;
	ot->ctx = ctx;
	*in_transfer = it;
	*out_transfer = ot;
	return 0;

free_it:
	free_transfer(it);
out:
	return -EINVAL;
}

/* Handle events generated by kernel and provided via ep0 */
int handle_ep0(int *ep, struct transfer *in_transfer, int *connected)
{
	struct usb_functionfs_event event;
	int ret;

	ret = read(ep[0], &event, sizeof(event));
	if (!ret) {
		report_error("unable to read event from ep0");
		return -EIO;
	}

	switch (event.type) {
	case FUNCTIONFS_SETUP:
		/* stall for all setuo requests */
		if (event.u.setup.bRequestType & USB_DIR_IN)
			(void) write(ep[0], NULL, 0);
		else
			(void) read(ep[0], NULL, 0);
		break;

	case FUNCTIONFS_ENABLE:
		*connected = 1;
		printf("Chat started. You may say something or type " EXIT_COMMAND
		       " to exit...\n");
		device_prompt = 1;
		printf("device> ");
		fflush(stdout);

		/*
		 * TODO: Start receiving messages from host
		 *
		 * int recv_message()
		 */
		ret = recv_message(in_transfer);
		if (ret < 0) {
			report_error("Unable to receive message");
			return ret;
		}
		break;

	case FUNCTIONFS_DISABLE:
		*connected = 0;
		break;

	default:
		break;
	}

	return 0;
}

/* main chat function */
void do_chat(int *ep, io_context_t *ctx, int event_fd)
{
	struct transfer *out_transfer;
	struct transfer *in_transfer;
	char *buf;
	int buf_size;
	fd_set rfds;
	int max_fd;
	int ret;
	int len;
	int wait_for_input = 1;
	int connected = 0;

	/* prepare our chat transfers */
	ret = prepare_transfers(ep, ctx, event_fd, &in_transfer, &out_transfer);
	if (ret) {
		report_error("Unable to prepare transfers");
		return;
	}

	/*
	 * We are on the device side so we use IN requests for sending
	 * data from device to host but still it is out transfer in our
	 * chat protocol
	 */
	buf = out_transfer->message.line_buf;
	buf_size = sizeof(out_transfer->message.line_buf);

	printf("Waiting for connection...\n");
	fflush(stdout);

	/* We cannot submit any transfer here as we may be not connected */

	/* our main program loop */
	while (1) {
		FD_ZERO(&rfds);
		/* we wait for input only if we have free out transfer */
		if (wait_for_input)
			FD_SET(STDIN_FILENO, &rfds);
		max_fd = STDIN_FILENO;

		/*
		 * We should wait for events only on ep0 and eventfd
		 * DONT add epX (x != 0) to poll()!
		 */
		FD_SET(ep[0], &rfds);
		max_fd = MAX(ep[0], max_fd);

		FD_SET(event_fd, &rfds);
		max_fd = MAX(event_fd, max_fd);

		/* we block here and wait for some events */
		ret = select(max_fd + 1, &rfds, NULL, NULL, NULL);
		if (ret < 0) {
			if (errno == EINTR)
				continue;
			report_error("Unable to use select");
			goto cleanup;;
		}

		/* first of all we check if we have some ep0 events */
		if (FD_ISSET(ep[0], &rfds)) {
			ret = handle_ep0(ep, in_transfer, &connected);
			if (ret)
				goto cleanup;
		}

		/*
		 * TODO: handle aio events if event_fd
		 * is ready for reading
		 *
		 * FD_ISSET()
		 * int handle_events()
		 */
		if (FD_ISSET(event_fd, &rfds)) {
			ret = handle_events(ctx, event_fd);
			if (ret)
				goto cleanup;
		}

		if (!connected)
			continue;
	}

cleanup:
	cancel_ffs_request(ctx, in_transfer->length_request);
	cancel_ffs_request(ctx, in_transfer->buf_request);
	cancel_ffs_request(ctx, out_transfer->length_request);
	cancel_ffs_request(ctx, out_transfer->buf_request);

	free_transfer(in_transfer);
	free_transfer(out_transfer);
}

int main(int argc, char **argv)
{
	int ep[3];
	int event_fd;
	io_context_t ctx;
	int ret;

	/* Check if we received ffs mount point */
	if (argc != 2) {
		printf("ffs directory not specified!\n");
		return 1;
	}
		
	report_error("preparing ffs in %s", argv[1]);

	ret = prepare_ffs(argv[1], ep);
	if (ret < 0) {
		report_error("Unable to prepare ffs: %d", ret);
		goto out;
	}

	while(1){
		sleep(1);	
	}

close_desc:
	cleanup_ffs(ep);
out:
	return ret;
}
```

Then we compile the program, which will be run by the earlier ```start-dji-usb-gadget.sh``` script:
```
gcc -Wall -O2 -c -o ~/startup-bulk.o ~/startup-bulk.c \
  && gcc ~/startup-bulk.o -o ~/startup-bulk -laio -flto
```

We clone and compile the PSDK code. 
```
git clone --depth 1 --recursive https://github.com/uzgit/Payload-SDK ~/git/Payload-SDK \
  && cd ~/git/Payload-SDK/ \
  && rm -rf build \
  && mkdir build \
  && cd build \
  && cmake .. \
  && make -j4 \
  && cd ~/git/Payload-SDK/
```

Before compiling (or before compiling for a second time), it is necessary to ensure that the PSDK configuration corresponds to the system's real setup per application.
For the E-port application, we are running the sample at ```samples/sample_c++/platform/linux/nvidia_jetson/```.
In ```application/dji_sdk_config.h```, we set
```
#define CONFIG_HARDWARE_CONNECTION         DJI_USE_UART_AND_USB_BULK_DEVICE
```
In ```hal/hal_uart.h```, we set ```#define LINUX_UART_DEV1    "/dev/ttyACM0"``` to make the PSDK application use ```uart0```.
In ```hal_usb_bulk.h``` we set the paths to the USB bulk device endpoints, interface numbers, and the Raspberry Pi's vendor ID (VID) and product ID (PID) as specified in the USB bulk script. We have the following parameters:
```
#define LINUX_USB_BULK1_EP_OUT_FD               "/dev/usb-ffs/bulk1/ep1"
#define LINUX_USB_BULK1_EP_IN_FD                "/dev/usb-ffs/bulk1/ep2"

#define LINUX_USB_BULK1_INTERFACE_NUM           (7)
#define LINUX_USB_BULK1_END_POINT_IN            (0x88)
#define LINUX_USB_BULK1_END_POINT_OUT           (5)

#define LINUX_USB_BULK2_EP_OUT_FD               "/dev/usb-ffs/bulk2/ep1"
#define LINUX_USB_BULK2_EP_IN_FD                "/dev/usb-ffs/bulk2/ep2"

#define LINUX_USB_BULK2_INTERFACE_NUM           (8)
#define LINUX_USB_BULK2_END_POINT_IN            (0x89)
#define LINUX_USB_BULK2_END_POINT_OUT           (6)

#ifdef PLATFORM_ARCH_x86_64
#define LINUX_USB_VID                         (0x0B95)
#define LINUX_USB_PID                         (0x1790)
#else
#define LINUX_USB_VID                         (0x0955)
#define LINUX_USB_PID                         (0x7020)
```

For the PSDK application, we run the application at ```src/rpi_desktop_streamer```,
we set the application to use a UART and network device in ```application/dji_sdk_config.h```:
```
#define CONFIG_HARDWARE_CONNECTION         DJI_USE_UART_AND_NETWORK_DEVICE
```
We set the following parameters in in ```hal/hal_uart.h```
```
#define LINUX_UART_DEV1    "/dev/ttyUSB0"
#define LINUX_UART_DEV2    "/dev/ttyUSB0"
```
since the application communicates with a USB-to-UART converter that is connected to the Raspberry Pi's USB ports.
We set the following parameters in ```hal/hal_network.h```
```
#ifdef PLATFORM_ARCH_x86_64
#define LINUX_NETWORK_DEV           "eth0"
#else
#define LINUX_NETWORK_DEV           "eth0"
#endif
```

For both applications, we have to set **unique** credentials in the ```dji_sdk_app_info.h``` file, which can be obtained via DJI's PSDK developer website: https://developer.dji.com/payload-sdk/apply/. 
```
#define USER_APP_NAME               "your_app_name"
#define USER_APP_ID                 "your_app_id"
#define USER_APP_KEY                "your_app_key"
#define USER_APP_LICENSE            "your_app_license"
#define USER_DEVELOPER_ACCOUNT      "your_developer_account"
#define USER_BAUD_RATE              "460800"
```
Once all of this information is correctly set, we recompile.

We had some trouble with the desktop streaming application freezing if we started it too early with respect to the Raspberry Pi's boot time. We therefore start it only after the Pi has been booted for 180 seconds.
We create the following script to delay startup:
```
#!/bin/bash

# Check if the number of arguments provided is correct
if [ $# -ne 1 ]; then
    echo "Usage: $0 <uptime_threshold_in_seconds>"
    exit 1
fi

# Store the uptime threshold provided as the first argument
n=$1

# Get the current uptime in seconds
uptime_seconds=$(awk '{print $1}' /proc/uptime)

# Check if the current uptime is less than n seconds
while (( $(echo "$uptime_seconds < $n" | bc -l) )); do
    # Calculate remaining time to sleep
    remaining_sleep=$(echo "$n - $uptime_seconds" | bc -l)
    
    # Print remaining time to sleep
    echo "Remaining time to reach minimum uptime: $remaining_sleep seconds"
    
    # Sleep for 5 seconds
    sleep 5
    
    # Update current uptime in seconds
    uptime_seconds=$(awk '{print $1}' /proc/uptime)
done

# Print the message
echo "Reached minimum uptime of $n seconds!"
```

And we create the following service file to handle automatically starting the desktop streamer application:
```
[Unit]
Description=Video Streamer Service
After=network.target

[Service]
Type=simple
User=root
Group=root
ExecStartPre=/home/joshua/reach_uptime.sh 180
ExecStart=/bin/bash -c "DISPLAY=:0 XAUTHORITY=/home/joshua/.Xauthority /home/joshua/git/Payload-SDK/build/bin/rpi_desktop_streamer"
TimeoutSec=300
Restart=always
RestartSec=3
StandardOutput=/home/joshua/log.txt

[Install]
WantedBy=multi-user.target
```

---
Below is the normal readme given by DJI.

# DJI Payload SDK (PSDK)

![](https://img.shields.io/badge/version-V3.8.1-purple.svg)
![](https://img.shields.io/badge/platform-linux_|_rtos-green.svg)
![](https://img.shields.io/badge/license-MIT-blue.svg)

## What is the DJI Payload SDK?

The DJI Payload SDK(PSDK), is a development kit provided by DJI to support developers to develop payload that can be
mounted on DJI drones. Combined with the X-Port, SkyPort or extension port adapter, developers can obtain the
information or other resource from the drone. According to the software logic and algorithm framework designed by the
developer, users could develop payload that can be mounted on DJI Drone, to perform actions they need, such as Automated
Flight Controller, Payload Controller, Video Image Analysis Platform, Mapping Camera, Megaphone And Searchlight, etc.

## Documentation

For full documentation, please visit
the [DJI Developer Documentation](https://developer.dji.com/doc/payload-sdk-tutorial/en/). Documentation regarding the
code can be found in the [PSDK API Reference](https://developer.dji.com/doc/payload-sdk-api-reference/en/)
section of the developer's website. Please visit
the [Latest Version Information](https://developer.dji.com/doc/payload-sdk-tutorial/en/)
to get the latest version information.

## Latest Release

The latest release version of PSDK is 3.8.1. This version of Payload SDK mainly add some new features support and fixed some
bugs. Please refer to the release notes for detailed changes list.

* Removed the camera management module interface DjiCameraManager_GetPhotoBurstCount.
* Removed the AEB photography function in the camera management module.
* Fixed the issue where subscribing to real-time point cloud data was not byte-aligned on some platforms.
* Fixed the discrepancy between flight speed units and annotations in the motion planning WP 2.0 feature.
* Fixed issues with obtaining the camera shooting mode and zoom magnification on the Mavic 3 series drones.
* Optimized the startup time for PSDK initialization.
* Complemented the flight control topic CONTROL_DEVICE data structure definition and added related enumeration ranges.
* Fixed the incorrect zoom multiplier retrieval for the H20N camera.
* Fixed the issue where custom media file suffix names were not taking effect.
* Fixed the occasional subscription errors for single battery data in the M300 RTK and M350 RTK.
* Fixed anomalous Pilot widget values for the M30 series, M300 RTK, and M350 RTK remote controllers.
* Note: Streamlined the toolchain for compiling PSDK, removing low-usage toolchain libraries.
> We have stopped offering some toolchains on GitHub. If you have trouble building your project, reach out to the SDK Support Team for help.

## License

Payload SDK codebase is MIT-licensed. Please refer to the LICENSE file for detailed information.

## Support

You can get official support from DJI and the community with the following methods:

- Post questions on Developer Forum
    * [DJI SDK Developer Forum(Cn)](https://djisdksupport.zendesk.com/hc/zh-cn/community/topics)
    * [DJI SDK Developer Forum(En)](https://djisdksupport.zendesk.com/hc/en-us/community/topics)
- Submit a request describing your problem on Developer Support
    * [DJI SDK Developer Support(Cn)](https://djisdksupport.zendesk.com/hc/zh-cn/requests/new)
    * [DJI SDK Developer Support(En)](https://djisdksupport.zendesk.com/hc/en-us/requests/new)

You can also communicate with other developers by the following methods:

- Post questions on [**Stackoverflow**](http://stackoverflow.com) using [**
  dji-sdk**](http://stackoverflow.com/questions/tagged/dji-sdk) tag

## About Pull Request
As always, the DJI Dev Team is committed to improving your developer experience, and we also welcome your contribution,
but the code review of any pull request maybe not timely, when you have any questionplease send an email to dev@dji.com.
