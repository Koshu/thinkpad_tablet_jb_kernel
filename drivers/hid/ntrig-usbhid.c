/*
 *  HID driver for N-Trig touchscreens
 *
 *  Copyright (c) 2011 N-TRIG 
 *
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/usb.h>

#include "typedef-ntrig.h"
#include "ntrig-common.h"
//#include "hid-ids.h"
#include "ntrig-dispatcher.h"
/**********************************************************************************/

struct ntrig_usbhid_data;
struct ntrig_usbhid_ncp;

/**
 * USB-HID interface API
 */
static int ntrig_usbhid_input_mapping	(	struct hid_device*	hdev, 
						struct hid_input*	hi,
						struct hid_field*	field, 
						struct hid_usage*	usage,
						unsigned long**		bit, 
						int*			max);
static int ntrig_usbhid_input_mapped	(	struct hid_device *hdev, 
						struct hid_input *hi, 
						struct hid_field *field, 
						struct hid_usage *usage, 
						unsigned long **bit, 
						int *max);
static int ntrig_usbhid_event		(	struct hid_device *hid, 
						struct hid_field *field, 
						struct hid_usage *usage, 
						__s32 value);
static int  ntrig_usbhid_probe		(	struct hid_device *hdev, const struct hid_device_id *id);
static void ntrig_usbhid_remove		(	struct hid_device *hdev);
static int  __init  ntrig_usbhid_init	(void);
static void __exit ntrig_usbhid_exit	(void);
static int ntrig_usbhid_reset_resume(	struct hid_device *hdev);


/**
 * Driver Host protocols API
 */
static int ntrig_usbhid_send_ncp_report	(	struct hid_device *hdev, const unsigned char *ncp_cmd);
static int ntrig_usbhid_send_report	(	struct hid_device *hdev, uint8_t request_report, const char *in_buf, short msg_len);

/**
 * Internal API for struct ntrig_usbhid_data
 */
void clear_input_buffer		(struct ntrig_usbhid_data* nd);
void set_button			(struct ntrig_usbhid_data* nd, __s32 btn);
void process_multi_touch_finger	(struct ntrig_usbhid_data* nd);
void send_multi_touch		(struct ntrig_usbhid_data* nd);
void send_pen			(struct ntrig_usbhid_data* nd);


/**********************************************************************************/
/*
 * N-trig HID Report Structure
 * The driver will support MTM firwmare Pen, Fingers
 */
struct ntrig_usbhid_data {
	lp_ntrig_bus_device 	ntrig_dispatcher;
	mr_message_types_t	message;
	__u16			pressure;
	__u8			events;
	__u16			x;
	__u16			y;
	__u16			frame_index;
	__u8			finger_id;
	__u16			dx;
	__u16			dy;
	__u8			generic_byte;
	__u8			blob_id;
	__u8			isPalm;
	__u8			msc_cnt;
	__s32			btn_pressed;
	__s32			btn_removed;
	__u8			first_occurance;
	__u8			sensor_id;
	__u8			battery_status;
	__u8			contact_count;
};

void clear_input_buffer(struct ntrig_usbhid_data* nd)
{
	memset(&nd->message, 0, sizeof(nd->message));
}

void set_button(struct ntrig_usbhid_data* nd, __s32 btn)
{
	if(nd->btn_pressed != btn){
		nd->btn_removed = nd->btn_pressed;		
		nd->btn_pressed = btn;
	}
}

void multi_touch_session_end(struct ntrig_usbhid_data* nd)
{	
	clear_input_buffer(nd);	
	nd->message.type	 			= MSG_FINGER_PARSE;
	nd->message.msg.fingers_event.num_of_fingers 	= 0;
	nd->message.msg.fingers_event.frame_index	= nd->frame_index;
	nd->message.msg.fingers_event.sensor_id 	= nd->sensor_id;	
	if (WriteHIDNTRIG(&nd->message) != DTRG_NO_ERROR)
		ntrig_dbg("FAILED to end MULTI-TOUCH session\n");
}

void process_multi_touch_finger(struct ntrig_usbhid_data* nd)
{	
	int index = nd->message.msg.fingers_event.num_of_fingers;

	if(0 == nd->finger_id){
		/* start preparing finger report */
		clear_input_buffer(nd);		
		nd->message.type = MSG_FINGER_PARSE;
		index = 0;
	}
	nd->message.msg.fingers_event.finger_array[index].track_id	= nd->finger_id;
	nd->message.msg.fingers_event.finger_array[index].x_coord	= nd->x;
	nd->message.msg.fingers_event.finger_array[index].y_coord	= nd->y;
	nd->message.msg.fingers_event.finger_array[index].dx		= nd->dx;
	nd->message.msg.fingers_event.finger_array[index].dy		= nd->dy;
	/* we pass generic byte via removed field */	
	nd->message.msg.fingers_event.finger_array[index].removed	= nd->generic_byte;
	nd->message.msg.fingers_event.finger_array[index].palm		= nd->isPalm;
	nd->message.msg.fingers_event.finger_array[index].generic	= nd->blob_id;
	nd->message.msg.fingers_event.num_of_fingers++;
}

void send_multi_touch(struct ntrig_usbhid_data* nd)
{
	/* lets verify that buffer is used for multi-touch data */
	if( MSG_FINGER_PARSE == nd->message.type) {
		nd->message.msg.fingers_event.num_of_fingers = nd->contact_count;
		/* add frame index, when  sending all fingers data */		
		nd->message.msg.fingers_event.frame_index = nd->frame_index;
		nd->message.msg.fingers_event.sensor_id   = nd->sensor_id;	
		if (WriteHIDNTRIG(&nd->message) != DTRG_NO_ERROR) 
			ntrig_dbg("FAILED to send MULTI-TOUCH\n");
	}
}

void send_pen(struct ntrig_usbhid_data* nd)
{
	clear_input_buffer(nd);		
	nd->message.type			= MSG_PEN_EVENTS;
	nd->message.msg.pen_event.x_coord	= nd->x;
	nd->message.msg.pen_event.y_coord	= nd->y;
	nd->message.msg.pen_event.pressure	= nd->pressure;
	nd->message.msg.pen_event.btn_code	= nd->btn_pressed;
	nd->message.msg.pen_event.btn_removed	= nd->btn_removed;
	nd->message.msg.pen_event.sensor_id 	= nd->sensor_id;	
	nd->message.msg.pen_event.battery_status = nd->battery_status;
	
	if (WriteHIDNTRIG(&nd->message) != DTRG_NO_ERROR)
		ntrig_dbg("FAILED to send PEN\n");
}
/**********************************************************************************/


/*
 * get feature buffer - as a result of sysfs show and set
 * first byte indicate amount of byte to copy
 */
static unsigned char hid_touch_ep_msg[HID_CLASS_TOUCH_EP_LEN];
static ntrig_usbhid_ncp_t ncp;
/*
 * this driver is aimed at two firmware versions in circulation:
 *  - dual pen/finger single touch
 *  - finger multitouch, pen not working
 */

static int ntrig_usbhid_input_mapping(	struct hid_device*	hdev, 
					struct hid_input*	hi,
					struct hid_field*	field, 
					struct hid_usage*	usage,
					unsigned long**		bit, 
					int*			max)
{
	struct ntrig_usbhid_data* nd = hid_get_drvdata(hdev);
	
	switch (usage->hid & HID_USAGE_PAGE) {
	case HID_UP_GENDESK:
		switch (usage->hid) {
		case HID_GD_X:
			hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_MT_POSITION_X);
			input_set_abs_params(hi->input, ABS_X, field->logical_minimum, field->logical_maximum, 0, 0);
			nd->ntrig_dispatcher->logical_min_x	= field->logical_minimum;
			nd->ntrig_dispatcher->logical_max_x	= field->logical_maximum;
			ntrig_dbg("%s ABX_X_MIN=%d ABX_X_MAX=%d\n", __FUNCTION__, field->logical_minimum, field->logical_maximum);
			return 1;
		case HID_GD_Y:
			hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_MT_POSITION_Y);
			nd->ntrig_dispatcher->logical_min_y	= field->logical_minimum;
			nd->ntrig_dispatcher->logical_max_y	= field->logical_maximum;
			input_set_abs_params(hi->input, ABS_Y, field->logical_minimum, field->logical_maximum, 0, 0);
			ntrig_dbg("%s ABX_Y_MIN=%d ABX_Y_MAX=%d\n", __FUNCTION__, field->logical_minimum, field->logical_maximum);
			return 1;
		}
		return 0;

	case HID_UP_DIGITIZER:
		switch (usage->hid) {
		/* we do not want to map these for now */
		case HID_DG_INVERT: /* Not support by pen */
		case HID_DG_ERASER: /* Not support by pen */

		case HID_DG_CONTACTID: /* Not trustworthy, squelch for now */
		case HID_DG_INPUTMODE:
		case HID_DG_DEVICEINDEX:
		case HID_DG_CONTACTMAX:
			return -1;
                /*case HID_DG_TIPSWITCH:
                        hid_map_usage(hi, usage, bit, max, EV_KEY, BTN_TOUCH);
                        return 1;*/
		/* width/height mapped on TouchMajor/TouchMinor/Orientation */
		case HID_DG_WIDTH:
			hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_MT_TOUCH_MAJOR);
			return 1;
		case HID_DG_HEIGHT:
			hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_MT_TOUCH_MINOR);
			hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_MT_TRACKING_ID);
			return 1;
		}
		return 0;

	case 0xff000000:
		/* we do not want to map these: no input-oriented meaning */
		return -1;
	}
	return 0;
}

/*
* This function maps Keys For Pen And Touch events
* MSC events used to transfer information about finger status
* In curent Frame
*/
static int ntrig_usbhid_input_mapped(	struct hid_device *hdev, 
					struct hid_input *hi,
					struct hid_field *field, 
					struct hid_usage *usage,
					unsigned long **bit, 
					int *max)
{
//	ntrig_dbg("%s \n", __FUNCTION__);

        if (usage->type == EV_KEY || usage->type == EV_ABS)
		clear_bit(usage->code, *bit);

	return 0;
}

/*
 * This function sends the DRIVER_ALIVE message to the touch screen upon waking
 * up after a device sleep.
 */
static int ntrig_usbhid_reset_resume(struct hid_device *hdev)
{
	int ret;
	ntrig_dbg( "inside %s \n", __FUNCTION__);
	ret = ntrig_usbhid_send_report(hdev, REPORTID_DRIVER_ALIVE, NULL, 0);
	if (ret < 0) {
		dev_err(&hdev->dev, "%s: send REPORTID_DRIVER_ALIVE failed, error code=%d\n", __FUNCTION__, ret);
	}
	return 0;
}
/*
 * this function is called upon all reports
 * so that we can filter contact point information,
 * decide whether we are in multi or single touch mode
 * and call input_mt_sync after each point if necessary
 */
static int ntrig_usbhid_event(struct hid_device *hid, struct hid_field *field,
			struct hid_usage *usage, __s32 value)
{
	struct usb_interface*		intf	 = to_usb_interface(hid->dev.parent);
	struct usb_host_interface*	interface= intf->cur_altsetting;
	struct usb_endpoint_descriptor*	endpoint = &interface->endpoint[0].desc;
	__s32				btn	 = BTN_UNPRESSED;	
	static __u16 ncp_report_counter = 0;

	/* ntrig_dbg("%s: us=%4X, ap=%4X, ep=%4X, cl=%4X val=%4X\n ", 
			  __FUNCTION__, usage->hid, field->application, endpoint->bEndpointAddress, hid->claimed, value); */
	if ( NCP_EP_ADDRESS == endpoint->bEndpointAddress ) {
	    switch (ncp_report_counter) {
		case NCP_HEADER: 
		  /* We Do Nothing with this, because each report as a diffrent start address */
		  break; 
		case NCP_HOSTADD_LSB: 
		  ncp.host_address = value;
		  break;
		case NCP_HOSTADD_MSB:
		  ncp.host_address |= value << 8;
		  break;
		case NCP_MSG_LEN_LSB: 
		  ncp.msg_length = value;
		  break;
		case NCP_MSG_LEN_MSB: 
		  ncp.msg_length |= value << 8;
		  break;
		case NCP_MSG_TYPE: 
		  ncp.msg_type = value;
		case NCP_CODE: 
		  ncp.msg_group_code = value;
		  break;
		case NCP_GROUP: 
		  ncp.msg_group_code |= value << 8;
		  break;
		case NCP_RTN_CODE_1:
		case NCP_RTN_CODE_2:
		case NCP_RTN_CODE_3:
		case NCP_RTN_CODE_4:
		    ncp.return_code[ncp_report_counter - 8] = value;
		    break;
		case NCP_RESERVED_LSB:
		    ncp.reserved = value;
		    break;
		case NCP_RESERVED_MSB:
		    ncp.reserved |= value << 8;
		    break;  
		default:
		    ncp.payload[ncp_report_counter -14] = value;
		  break;
	    }
	    if (++ncp_report_counter == field->report_count)
		ncp_report_counter = 0;
	    return 1;
	}
	if (hid->claimed & HID_CLAIMED_INPUT) {
	    struct ntrig_usbhid_data*	nd	= hid_get_drvdata(hid);
	    struct input_dev*		input	= field->hidinput->input;

	    /* We attach multi touch to dispatcher if there is no multi touch queue*/
	    if(!check_multi_touch(nd->sensor_id)) {
		ntrig_dbg("%s Attach Multi Touch device to dispatcher\n", __FUNCTION__);
		attach_multi_touch(nd->sensor_id, input);
	    }	

	    /* We attach multi touch to dispatcher if there is no single touch queue */
	    if(!check_single_touch(nd->sensor_id)) {
		ntrig_dbg("%s Attach Single Touch device to dispatcher\n", __FUNCTION__);
		attach_single_touch(nd->sensor_id, input);
	    }	

	    switch (usage->hid) {
		case HID_GD_X:
		/* ntrig_dbg("%s: HID_GD_X=%d\n", __FUNCTION__, value); */
		nd->x = value;
		break;
	    case HID_GD_Y:
		/* ntrig_dbg("%s: HID_GD_Y=%d\n", __FUNCTION__, value); */
		nd->y = value;
		break;
	    }
	    if (field->application == HID_DG_PEN) {
		switch (usage->hid) {
			case HID_DG_INRANGE:
//	ntrig_dbg("%s: HID_DG_PEN: HID_DG_INRANGE=%x, value %d\n", __FUNCTION__, usage->hid, value);
				nd->events = value;
				break;
			case HID_DG_TIPSWITCH:
//	ntrig_dbg("%s: HID_DG_PEN: HID_DG_TIPSWITCH=%x, value %d\n", __FUNCTION__, usage->hid, value);
				nd->events |= (value << 1);
				break;
			case HID_DG_BARRELSWITCH:
//	ntrig_dbg("%s: HID_DG_PEN: HID_DG_BARRELSWITCH=%x, value %d\n", __FUNCTION__, usage->hid, value);
				nd->events |= (value << 2);
				break;
			case HID_DG_INVERT:
//	ntrig_dbg("%s: HID_DG_PEN: HID_DG_INVERT=%x, value %d\n", __FUNCTION__, usage->hid, value);
				nd->events |= (value << 3);
				break;
			case HID_DG_ERASER:
//	ntrig_dbg("%s: HID_DG_PEN: HID_DG_ERASER=%x, value %d\n", __FUNCTION__, usage->hid, value);
				nd->events |= (value << 4);
				break;
			case HID_DG_TIPPRESSURE:
//	ntrig_dbg("%s: HID_DG_PEN: HID_DG_TIPPRESSURE=%x, value %d\n", __FUNCTION__, usage->hid, value);
				nd->pressure = value;
				btn = (int) nd->events;
				/* process button information and send to dispatcher if required */			
				set_button(nd, btn); 
				/* send pen data to dispatcher */ 
				send_pen  (nd);	   
				/* print button and pen data */
//				ntrig_dbg("Privet X=%d Y=%d Button=%d Pressure=%d\n", 
//					nd->x, 
//					nd->y, 
//					nd->btn_pressed, 
//					nd->pressure);
				break;
#if 0
			case PEN_BATTERY_STATUS:
//	ntrig_dbg("%s: HID_DG_PEN: PEN_BATTERY_STATUS=%x, value %d\n", __FUNCTION__, usage->hid, value);
				if (nd->events & 0x01)	// TIPSWITCH == true --> pen is touching the screen --> battery_status value is valid
					nd->battery_status = value;
				else			// TIPSWITCH == false --> pen is not touching the screen --> battery_status value is not valid
					nd->battery_status = PEN_BUTTON_BATTERY_NOT_AVAILABLE;
	ntrig_dbg("%s: HID_DG_PEN: PEN_BATTERY_STATUS=%x, value %d, batter_status = %d, \n", __FUNCTION__, usage->hid, value, nd->battery_status);
				break;
#endif
			default:
//				ntrig_dbg("%s: HID_DG_PEN: default=%x, value %d\n", __FUNCTION__, usage->hid, value);
				break;
		}
	   } else { /* MultiTouch Report */
			switch (usage->hid) {
			case HID_DG_CONTACTCOUNT:
				nd->contact_count = value;
				/* end of report (this field always comes last) - send to dispatcher*/
				send_multi_touch(nd);
				break;
			case MTM_FRAME_INDEX: /* Index 1 */
//				ntrig_dbg("%s: MultiTouch Report: MTM_FRAME_INDEX=%x, value %d\n", __FUNCTION__, usage->hid, value);
				nd->frame_index = value;
				break;
			case HID_DG_TIPSWITCH:	/* FALLTHRU */
			case HID_DG_INRANGE:	/* FALLTHRU */
			case HID_DG_CONFIDENCE: /* Not Relevant-Index 2 - 4 */
				break;
			case HID_DG_CONTACTID: /* Index 5 */
				nd->finger_id = value;
				break;
			case HID_DG_WIDTH:/* Index 6 - 7*/
				nd->dx = value;
				break;
			case HID_DG_HEIGHT:/* Index 8 - 9 */
				nd->dy = value;
				/* Start The Sequence of MSC bytes */
				nd->msc_cnt = 0;
				break;
			case MTM_PROPRIETARY:/* Index 10 - 14 */
//				ntrig_dbg("%s: MultiTouch Report: v=%x, value %d\n", __FUNCTION__, usage->hid, value);
				nd->msc_cnt++;
//				ntrig_dbg("%s: MTM_PROPRIETARY msc_cnt=%d val=%d\n", __FUNCTION__, nd->msc_cnt, value);
				switch (nd->msc_cnt) {
				case REPORT_GENERIC1:
					nd->generic_byte = value;
					break;
				case REPORT_MT:
					nd->blob_id = value;
					break;
				case REPORT_PALM:
					nd->isPalm = value;
					break;
				case REPORT_GENERIC2:
					/* end of single finger part of report */
					 /* process multi touch finger data */
					process_multi_touch_finger(nd);
					/* Print finger data */						
					ntrig_dbg("Frame=%x Finger=%d X=%d Y=%d DX=%d DY=%d FirstOccur=%d Palm=%d Press=%d Blob=%d\n",
							nd->frame_index, 
							nd->finger_id, 
							nd->x,
							nd->y, 
							nd->dx, 
							nd->dy,
							nd->generic_byte, 
							nd->isPalm, 
							nd->pressure,
							nd->blob_id);
					break;
				}
				break;
			}
		}
	}
	/* we have handled the hidinput part, now remains hiddev */
	if ((hid->claimed & HID_CLAIMED_HIDDEV) && hid->hiddev_hid_event)
		hid->hiddev_hid_event(hid, field, usage, value);
	return 1;
}

/*
 * This function used to configure N-trig firmware
 * The first command we need to send to firmware is change
 * to Multi-touch Mode we don't receive a reply
 * Endpoint 1 NCP 
 */
static int ntrig_usbhid_send_ncp_report(struct hid_device *hdev, const unsigned char *ncp_cmd)
{
	struct hid_report *report;
	struct list_head *report_list =
			&hdev->report_enum[HID_FEATURE_REPORT].report_list;

	struct usb_interface *intf = to_usb_interface(hdev->dev.parent);
        struct usb_device *dev = interface_to_usbdev(intf);
        __u16 ifnum = intf->cur_altsetting->desc.bInterfaceNumber;
        int ret;
	
	ntrig_dbg( "inside %s \n", __FUNCTION__);
	ntrig_dbg("%s: ncp_cmd[0]=%0X ncp_cmd[1]=%0X\n", __FUNCTION__, ncp_cmd[0], ncp_cmd[1]);
	report = list_first_entry(report_list, struct hid_report, list);
	if (report->maxfield < 1)
		return -ENODEV;
	list_for_each_entry(report,
			    report_list, list) {
		if (report->maxfield < 1) {
		      ntrig_dbg("no fields in the report\n");
		      continue;
		}
		/*
		* Bytes Needed From User
		* 0 - Length Of Message
		* 1 - Reprot ID
		* .. - Data
		*/
		/* ntrig_dbg("%s: checking report id %d for candidate %d\n", __FUNCTION__, report->id, ncp_cmd[1]); */
		if (ncp_cmd[HID_NCP_CMD_DATA] == report->id) {
		    char *buf = kmalloc(ncp_cmd[HID_NCP_CMD_LEN], GFP_KERNEL);
		    if (!buf)
			return -ENOMEM;
		    memcpy(buf, &ncp_cmd[HID_NCP_CMD_DATA], ncp_cmd[HID_NCP_CMD_LEN]); 
		    ret = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
                                 HID_REQ_SET_REPORT ,
                                 USB_DIR_OUT | USB_TYPE_CLASS |
                                 USB_RECIP_INTERFACE,
                                 ( 0x03 << 8) | ncp_cmd[HID_NCP_CMD_DATA], ifnum, buf, ncp_cmd[HID_NCP_CMD_LEN],
                                 USB_CTRL_SET_TIMEOUT);

		    kfree(buf);
		}
	}
	return 0;
}

/*
 * This function is used to configure N-trig firmware
 * The first command we need to send to firmware is change
 * to Multi-touch Mode we don't receive a reply
 * Endpoint 2 - Touch/Pen events
 */
static int ntrig_usbhid_send_report(struct hid_device *hdev, uint8_t request_report, const char *in_buf, short msg_len)
{
	struct hid_report *report;
	struct list_head *report_list =
			&hdev->report_enum[HID_FEATURE_REPORT].report_list;

	struct usb_interface *intf = to_usb_interface(hdev->dev.parent);
        struct usb_device *dev = interface_to_usbdev(intf);
        __u16 ifnum = intf->cur_altsetting->desc.bInterfaceNumber;
        int ret = DTRG_FAILED;

	ntrig_dbg( "inside %s \n", __FUNCTION__);
	
	report = list_first_entry(report_list, struct hid_report, list);
	if (report->maxfield < 1)
		return -ENODEV;

	list_for_each_entry(report,
			    report_list, list) {
		if (report->maxfield < 1) {
		      ntrig_dbg("no fields in the report\n");
		      continue;
		}
		if (request_report == report->id) {
			int bufsize = (msg_len < HID_CLASS_TOUCH_EP_LEN) ? HID_CLASS_TOUCH_EP_LEN : msg_len;
		    char *buf = kmalloc(bufsize, GFP_KERNEL);
		    if (!buf)
			return -ENOMEM;
		    if (!msg_len) { // msg_len == 0 ==> get report
			    ret = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0),
		                         HID_REQ_GET_REPORT,
		                         USB_DIR_IN | USB_TYPE_CLASS |
		                         USB_RECIP_INTERFACE,
		                         (0x03 << 8) | request_report, ifnum, buf, report->size,
		                         USB_CTRL_GET_TIMEOUT);
			    hid_touch_ep_msg[HID_REPLY_SIZE] = ret;
			    if (ret >= 0) // success
			    	memcpy(&hid_touch_ep_msg[HID_REPLY_DATA], buf, ret);
		    }
		    else { // msg_len > 0 ==> set report
			    if (!in_buf) {
				kfree(buf);
				return -ENOMEM;
			    }
		    	    memcpy(buf, in_buf, msg_len); 
			    ret = usb_control_msg(dev, usb_sndctrlpipe(dev, 0),
					HID_REQ_SET_REPORT ,
                                 	USB_DIR_OUT | USB_TYPE_CLASS |
                                 	USB_RECIP_INTERFACE,		                        
					(0x03 << 8) | request_report, ifnum, buf, msg_len,
		                        USB_CTRL_SET_TIMEOUT);
 		    }
		    kfree(buf);
		}
	}
	return ret;
}

/** 
 * Write sensor configuration msg to HID device. 
 * Return number of bytes written/read or <0 if failed.
 */
static __u8 next_read_sensor_command = -1;

int NTRIGWriteSensor (void *dev, uint8_t cmd, const char *in_buf, short msg_len)
{
	int retval; // holds the number of bytes written/read or <0 if failed, so no need for the above check	

	ntrig_dbg( "inside %s cmd = %d\n", __FUNCTION__, cmd);
	
	if (!dev) {
		ntrig_dbg("%s: wrong paramas\n", __FUNCTION__);
		return DTRG_FAILED;
	}

	// Get battery status from SW var, not from device
	if (cmd == REPORTID_GET_PEN_BATTRY_STATUS) {
		next_read_sensor_command = REPORTID_GET_PEN_BATTRY_STATUS;	
//		ntrig_dbg("%s: REPORTID_GET_PEN_BATTRY_STATUS: next command = %d \n", __FUNCTION__, next_read_sensor_command);
		return 1;
	}

	retval = ntrig_usbhid_send_report((struct hid_device *)dev, cmd, in_buf, msg_len);

	ntrig_dbg( "Leaving %s , count = %d, msg_len = %d\n", __FUNCTION__, retval, msg_len);

	if(retval < 0) {
		/* failed */
		return retval;
	}

	return msg_len + 1; // add 1 for command
}

/** 
 * Read sensor configuration response from HID device. 
 * Return number of bytes read or fail.
 */
int NTRIGReadSensor (void *dev, char *buf, size_t count)
{
	int size = hid_touch_ep_msg[HID_REPLY_SIZE];
	struct ntrig_usbhid_data* nd = hid_get_drvdata((struct hid_device *)dev);

	ntrig_dbg( "usbhid: inside %s out_buf size = %d \n", __FUNCTION__, (int)sizeof(buf));
	// Get battery status from SW var, not from device
	if (next_read_sensor_command == REPORTID_GET_PEN_BATTRY_STATUS) {
		buf[0] = (char) nd->battery_status;
//		ntrig_dbg("%s: REPORTID_GET_PEN_BATTRY_STATUS: status = %d \n", __FUNCTION__, buf[0]);
		next_read_sensor_command = -1;	
		return 1;
	}
	
	/* size of input buffer (coming from sysfs show function) is PAGE_SIZE (4096 bytes)
	   so it's very unlikely we'll have memory overwriting. 
	   Therefore there's no need to check the buf size before writing to it
	*/
	if (count < size) {	// buffer too small
		ntrig_dbg( "usbhid: Leaving %s, count (%d)< size(%d)\n", __FUNCTION__, (int)count, size);
		return -1;
	}

	memcpy(buf, &hid_touch_ep_msg[HID_REPLY_DATA], size);
	ntrig_dbg( "usbhid: Leaving %s \n", __FUNCTION__);

	return size;
}

/** 
 * Write NCP msg to HID device. 
 * Return success or fail.
 */
int NTRIGWriteNCP (void *dev, const char *buf, short msg_len)
{
	int count = 0;
	struct hid_device *hdev = dev;

	ntrig_dbg( "inside %s \n", __FUNCTION__);
	ntrig_dbg("%s: buf[0]=%0X buf[1]=%0X\n", __FUNCTION__, buf[0], buf[1]);
	
	if ((!dev) || (!buf) || (msg_len == 0)) {
		ntrig_dbg("%s: wrong paramas\n", __FUNCTION__);
		return DTRG_FAILED;
	}

	//This version don't support bytes received from shell (ascii)
	ntrig_usbhid_send_ncp_report(hdev, buf);

	ntrig_dbg( "Leaving %s , count = %d\n", __FUNCTION__, count);	
	return count;
}

/** 
 * Read NCP msg from HID device. 
 * Return number of bytes read or fail.
 */
int NTRIGReadNCP (void *dev, char *buf, size_t count)
{
	int size = sizeof(ntrig_usbhid_ncp_t);

	ntrig_dbg( "usbhid: inside %s \n", __FUNCTION__);
	
	/* allocated buffer too small */
	if (count < size)
		return DTRG_FAILED;

	/* buf is allocated by dispatcher, therefore no need to call copy_to_user */
	memcpy(buf, &ncp, size);
	return size;
}


static int ntrig_usbhid_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	char				phys_path[256];
	int 				ret;
	struct ntrig_usbhid_data*	nd;
	struct usb_interface*		intf;
	struct usb_device* 		dev;
	struct usb_host_interface*	interface;
	struct usb_endpoint_descriptor*	endpoint;
	struct _ntrig_dev_ncp_func	ncp_func;
	struct _ntrig_dev_hid_func	hid_func;


	intf 	 = to_usb_interface(hdev->dev.parent);
	dev	 = interface_to_usbdev(intf);
	interface= intf->cur_altsetting;
	endpoint = &interface->endpoint[0].desc;
	nd = kmalloc(sizeof(struct ntrig_usbhid_data), GFP_KERNEL);
	if (!nd) {
		dev_err(&hdev->dev, "cannot allocate N-Trig data\n");
		return -ENOMEM;
	} 
	if(DTRG_NO_ERROR != allocate_device(&nd->ntrig_dispatcher)){
		dev_err(&hdev->dev, "cannot allocate N-Trig dispatcher\n");
		return DTRG_FAILED;
	}

	hid_set_drvdata(hdev, nd);
	ret = hid_parse(hdev);
	if (ret) {
		dev_err(&hdev->dev, "parse failed, error code=%d\n", ret);
		goto err_free;
	}

	if (NCP_EP_ADDRESS == endpoint->bEndpointAddress) {
		ret = hid_hw_start(hdev, ~HID_QUIRK_MULTI_INPUT);
		if (ret) {
			dev_err(&hdev->dev, "hw start failed NCP, error code=%d\n", ret);
			goto err_free;
		}

		/* register device in the dispatcher - set device and function pointers for ncp
		set sensor id in the device data structure */	  
	// NOTE: registration of ncp_read, ncp_write & dev is linked. 
	//  It's the resposibility of this function to set these parameters together
		ncp_func.dev = (void *) hdev;
		ncp_func.read = NTRIGReadNCP;
		ncp_func.write = NTRIGWriteNCP;
		ncp_func.read_counters = NULL;//counters not implemented in USB driver
		ncp_func.reset_counters = NULL;//counters not implemented in USB driver
		nd->sensor_id = RegNtrigDispatcher(TYPE_BUS_USB_HID, hdev->uniq, &ncp_func, NULL);
		// TODO: define behavior in case of fail
		if (nd->sensor_id == DTRG_FAILED) {
			ntrig_dbg("%s: Cannot register device to dispatcher\n", __FUNCTION__);
			goto err_free;
		}
		ntrig_dbg("%s NCP dev registered with dispatcher, bus_id = %s, sensor_id = %d\n", __FUNCTION__, hdev->uniq, nd->sensor_id);
		return 0;
	}

	else {
		ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT & ~HID_CONNECT_FF);
		if (ret) {
			dev_err(&hdev->dev, "hw start failed TOUCH, error code=%d\n", ret);
			goto err_free;
		}
		ret = ntrig_usbhid_send_report(hdev, REPORTID_DRIVER_ALIVE, NULL, 0);
		if (ret < 0) {
			dev_err(&hdev->dev, "send set feature failed, error code=%d\n", ret);
			goto err_free;
		}

		/* register device in the dispatcher - no need to set device and function pointers for touch events
		   set sensor id in the device data structure */	  
	// NOTE: registration of ncp_read, ncp_write & dev is linked. 
	//  It's the resposibility of this function to set these parameters together
		hid_func.dev = (void *)hdev;
		hid_func.read = NTRIGReadSensor;
		hid_func.write = NTRIGWriteSensor;
		nd->sensor_id = RegNtrigDispatcher(TYPE_BUS_USB_HID, hdev->uniq, NULL, &hid_func); 
		// TODO: define behavior in case of fail
		if (nd->sensor_id == DTRG_FAILED) {
			ntrig_dbg("%s: Cannot register device to dispatcher\n", __FUNCTION__);
			goto err_free;
		}
		ntrig_dbg("%s HID dev registered with dispatcher, bus_id = %s, sensor_id = %d\n", __FUNCTION__, hdev->uniq, nd->sensor_id);
	}
	/**
	 * Create additional single touch queue
         */

	usb_make_path(dev, phys_path, sizeof(phys_path));
	strlcat(phys_path, "/input0", sizeof(phys_path));
	
	nd->ntrig_dispatcher->phys = phys_path;
	nd->ntrig_dispatcher->name = "USBHID";
	nd->ntrig_dispatcher->pressure_min= 0;
	nd->ntrig_dispatcher->pressure_max= 0xFF;
	create_single_touch(nd->ntrig_dispatcher, nd->sensor_id);
	create_multi_touch (nd->ntrig_dispatcher, nd->sensor_id);

	nd->battery_status = PEN_BUTTON_BATTERY_NOT_AVAILABLE;

	ntrig_dbg("Iside %s, bus_id = %d, name = %s, phys = %s, uniq = %s\n", __FUNCTION__, hdev->bus, hdev->name, hdev->phys, hdev->uniq );

	ntrig_dbg("End of %s\n", __FUNCTION__);
	return DTRG_NO_ERROR;
err_free:
	ntrig_dbg("Error End of %s\n", __FUNCTION__);
	remove_device(&nd->ntrig_dispatcher);	
	kfree(nd);
	return DTRG_FAILED;
}

static void ntrig_usbhid_remove(struct hid_device *hdev)
{
	struct ntrig_usbhid_data*	nd;

	ntrig_dbg("Entering %s\n", __FUNCTION__);
	hid_hw_stop(hdev);

	nd = hid_get_drvdata(hdev);
	UnregNtrigDispatcher(nd->ntrig_dispatcher, nd->sensor_id, TYPE_BUS_USB_HID, hdev->uniq);	
	remove_device(&nd->ntrig_dispatcher);		
	kfree(nd);

	ntrig_dbg("%s all resource released\n", __FUNCTION__);
}

static const struct hid_device_id ntrig_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_NTRIG, USB_DEVICE_ID_NTRIG_TOUCH_SCREEN),
		.driver_data = NTRIG_USB_DEVICE_ID },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NTRIG, USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_1),
		.driver_data = NTRIG_USB_DEVICE_ID },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NTRIG, USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_2),
		.driver_data = NTRIG_USB_DEVICE_ID },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NTRIG, USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_3),
		.driver_data = NTRIG_USB_DEVICE_ID },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NTRIG, USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_4),
		.driver_data = NTRIG_USB_DEVICE_ID },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NTRIG, USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_5),
		.driver_data = NTRIG_USB_DEVICE_ID },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NTRIG, USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_6),
		.driver_data = NTRIG_USB_DEVICE_ID },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NTRIG, USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_7),
		.driver_data = NTRIG_USB_DEVICE_ID },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NTRIG, USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_8),
		.driver_data = NTRIG_USB_DEVICE_ID },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NTRIG, USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_9),
		.driver_data = NTRIG_USB_DEVICE_ID },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NTRIG, USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_10),
		.driver_data = NTRIG_USB_DEVICE_ID },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NTRIG, USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_11),
		.driver_data = NTRIG_USB_DEVICE_ID },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NTRIG, USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_12),
		.driver_data = NTRIG_USB_DEVICE_ID },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NTRIG, USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_13),
		.driver_data = NTRIG_USB_DEVICE_ID },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NTRIG, USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_14),
		.driver_data = NTRIG_USB_DEVICE_ID },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NTRIG, USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_15),
		.driver_data = NTRIG_USB_DEVICE_ID },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NTRIG, USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_16),
		.driver_data = NTRIG_USB_DEVICE_ID },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NTRIG, USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_17),
		.driver_data = NTRIG_USB_DEVICE_ID },
	{ HID_USB_DEVICE(USB_VENDOR_ID_NTRIG, USB_DEVICE_ID_NTRIG_TOUCH_SCREEN_18),
		.driver_data = NTRIG_USB_DEVICE_ID },
	{ }
};
MODULE_DEVICE_TABLE(hid, ntrig_devices);

static struct hid_driver ntrig_usbhid_driver = {
	.name = "ntrig_usbhid",
	.id_table = ntrig_devices,
	.probe = ntrig_usbhid_probe,
	.remove = ntrig_usbhid_remove,
	.input_mapping = ntrig_usbhid_input_mapping,
	.input_mapped = ntrig_usbhid_input_mapped,
	.event = ntrig_usbhid_event,
	.reset_resume = ntrig_usbhid_reset_resume,
};

static int __init ntrig_usbhid_init(void)
{
	printk("USBHID Driver Version 2.00\n");//DEBUG Use untill we stablize the version
	return hid_register_driver(&ntrig_usbhid_driver);
}

static void __exit ntrig_usbhid_exit(void)
{
	hid_unregister_driver(&ntrig_usbhid_driver);
}

module_init(ntrig_usbhid_init);
module_exit(ntrig_usbhid_exit);

MODULE_LICENSE("GPL");
