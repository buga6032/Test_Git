#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/ioctl.h>
#include <pthread.h>
#include <unistd.h>

#include "libusbi.h" 
#include "ys_libusb.h"
#include "ys_libusb_version.h"
//#include "key_verify.h"


#if 0 //CRC_ENABLE
static void print_num(unsigned char *data,int count)
{
	int i;
	ysprint("\n");
	for(i=0;i<count;i++){
		ysprint("%d\t",data[i]);
		if((i+1)%20==0)ysprint("\n");
	}
	ysprint("\n");
}
#endif

//#ifdef YS_PRINT_API
int (*ysprint)(const char *fmt, ...) = printf;

int libysusb_set_print_funcp(int (*func)(const char *fmt, ...))
{
	if (!func)
		return -1;

	ysprint = func;

	return 0;
}
//#endif /* YS_PRINT_API */

static unsigned char get_crc(unsigned char *data,int len)
{
	unsigned char value = 0;
	int i;

	for (i = 0; i < len; i ++) {
		value = (unsigned char)(value+ data[i]);
	}

	return value;
}


/**********************************************
 函数说明：USB打开设备函数，成功返回找到的设备数，失败返回负数。
 port是一个数组，存放设备的端口。
 handle是一个数组，打开的USB句柄依次存放在这个数组里面。
**********************************************/ 
int usb_open(struct ys_usb_handle *handle,int *port)
{
	int i,ret,cnt,portnum,num=0;
	libusb_device **devs,*tmp;
	struct libusb_device_descriptor desc;
	struct libusb_config_descriptor *config;

#if CRC_ENABLE
	ysprint("NEW USB driver lib on ARM,YS_LIBUSB_VERSION = %d,MaxPacketSize = %d,CRC enable.\n",YS_LIBUSB_VERSION,MaxPacketSize);
#else
	ysprint("NEW USB driver lib on ARM,YS_LIBUSB_VERSION = %d,MaxPacketSize = %d,CRC disable.\n",YS_LIBUSB_VERSION,MaxPacketSize);
#endif

	ret = libusb_init(NULL);
	if (ret < 0) {
		ysprint("initial USB lib failed! ret = %d\n",ret);
		return -1;
	} 

	cnt = libusb_get_device_list(NULL, &devs);
	if (cnt < 0) { //swj add
		ysprint("%s:%d %s: Failed to get a list of connected devices, retval %d\n", __FILE__, __LINE__, __FUNCTION__, cnt);
		return cnt;
	}

	for(i=0;i<cnt;i++)
	{
		memset(&desc,0,sizeof(struct libusb_device_descriptor));
		ret = libusb_get_device_descriptor(devs[i], &desc);
		if(ret<0){
			ysprint("USB device get device descriptor failed,i=%d,cnt=%d\n",i,cnt);
			continue;
		}
		//ysprint("count = %d,idVendor = %x\n",i,desc.idVendor);

		if(desc.idVendor==YS_USB_VENDOR_ID){
			tmp = devs[i];
			//ysprint("portnum = %d\n",tmp->port_number);
			portnum = tmp->port_number - 1;
			port[num] = portnum;

			//handle[portnum].libusb_handle = libusb_open_device_with_vid_pid(NULL , desc.idVendor , desc.idProduct);
			libusb_open(devs[i],&(handle[portnum].libusb_handle));
			if(handle[portnum].libusb_handle==NULL){
				ysprint("USB device open failed,the USB PID = %x\n",desc.idProduct);
				continue;
			}
			handle[portnum].idproduct = desc.idProduct;

			ret = libusb_set_configuration(handle[portnum].libusb_handle,1);
			if(ret<0){
				ysprint("USB device set config failed,the USB PID = %x\n",desc.idProduct);
				continue;
			}

			ret = libusb_claim_interface(handle[portnum].libusb_handle,0);
			if(ret<0){
				ysprint("USB device claim interface failed,the USB PID = %x\n",desc.idProduct);
				continue;
			}

			ret = libusb_get_config_descriptor(devs[i],0,&config);
			if(ret<0){
				ysprint("USB device get config descriptor failed,the USB PID = %x\n",desc.idProduct);
				libusb_free_config_descriptor(config);
				continue;
			}
			else {
				handle[portnum].Endpoint_in = config->interface->altsetting->endpoint[0].bEndpointAddress;
				handle[portnum].Endpoint_out = config->interface->altsetting->endpoint[1].bEndpointAddress;
			}
			libusb_free_config_descriptor(config);

			//pthread_mutex_init(&handle[portnum].lock, NULL);
			//pthread_mutex_init(&handle[portnum].rwlock, NULL);
			num++;
		}
	}

	libusb_free_device_list(devs, 1);

	if(num==0)
		return ret;
	else 
		return num;
}

/**********************************************
 函数说明：USB写函数，成功返回0，失败返回负数。count
 是要写的字节数。支持多个线程同时写操作。
**********************************************/
int usb_write(struct ys_usb_handle *handle,unsigned char *data,int count)
{
	unsigned char *txbuf;
	int i,ret,actual_length;//,times,realsize;
	int totalsize,packetsize,packetnum,headersize,offset1,offset2;
	struct usb_driver_header header;

#if CRC_ENABLE
	int crc_size;
#endif

//	pthread_mutex_lock(&(handle->rwlock));

//	pthread_mutex_lock(&(handle->lock));

	headersize = sizeof(struct usb_driver_header);
	packetsize = MaxPacketSize - headersize;

	if(count%packetsize==0) packetnum = count/packetsize;
	else packetnum = count/packetsize + 1;

	totalsize = count+packetnum*headersize;

	txbuf = malloc(totalsize);
	if(txbuf==NULL){
		ysprint("usb_write() malloc buffer failed!\n");
		return -1;
	}

	offset1 = 0;
	offset2 = 0;
	header.length = packetnum;

	for(i=0;i<packetnum;i++){
		header.num = i+1;
		header.size = (count-offset2)>packetsize? packetsize:(count-offset2) + sizeof(struct usb_driver_header);

#if CRC_ENABLE
		crc_size = (count-offset2)>packetsize? packetsize:(count-offset2);
		header.crc = get_crc(data+offset2,crc_size);
		//ysprint("header.crc = %d,data[%d] = %d\n",header.crc,offset2,data[offset2]);
#endif
		memcpy(txbuf+offset1,&header,sizeof(struct usb_driver_header));
		offset1 += headersize;
		memcpy(txbuf+offset1,data+offset2,(count-offset2)>packetsize? packetsize:(count-offset2));
		offset2 += packetsize;
		offset1 += packetsize;
	}

#if 0
	if(totalsize%MAX_BULK_LENGTH==0)times = totalsize/MAX_BULK_LENGTH;
	else times = totalsize/MAX_BULK_LENGTH + 1;

	offset1 = 0;
	for(i=0;i<times;i++){
		if(i==(times-1)){
		    realsize = totalsize - MAX_BULK_LENGTH*i;
		    ret = libusb_bulk_transfer(handle->libusb_handle,handle->Endpoint_out,txbuf+offset1,realsize,&actual_length,0);
		    if(ret<0){
			    ysprint("USB write data failed,ret = %d,USB idProduct = %x\n",ret,handle->idproduct);
		    }else{
			    actual_length = actual_length - packetnum*headersize;
			    if(actual_length!=count){
				    ysprint("USB write count not equal actual length\n");
				    ret = -1;
			    }
		    }
		}else {
		    realsize = MAX_BULK_LENGTH;
		    ret = libusb_bulk_transfer(handle->libusb_handle,handle->Endpoint_out,txbuf+offset1,realsize,&actual_length,0);
		    if(ret<0){
			    ysprint("USB write data failed,ret = %d,USB idProduct = %x\n",ret,handle->idproduct);
		    }else{
			    actual_length = actual_length - packetnum*headersize;
			    if(actual_length!=count){
				    ysprint("USB write count not equal actual length\n");
				    ret = -1;
			    }
		    }	   
		}
		offset1 += realsize;
	}

#else
    ret = libusb_bulk_transfer(handle->libusb_handle,handle->Endpoint_out,txbuf,totalsize,&actual_length,0);
    if (ret < 0) {
		ysprint("USB write data failed,ret = %d,USB idProduct = %x\n",ret,handle->idproduct);
    } else {
		actual_length = actual_length - packetnum*headersize;
		if (actual_length != count) {
			ysprint("USB write count not equal actual length\n");
			ret = -1;
		}
    }
#endif
	 
	if(txbuf!=NULL){
		free(txbuf);
		txbuf = NULL;
	}

//	pthread_mutex_unlock(&(handle->lock));

//	pthread_mutex_unlock(&(handle->rwlock));
	return ret;
}

/**********************************************
 函数说明：USB读函数，读不到数据阻塞。成功返回读到
 的字节数，失败返回-1。count是要读取的字节数，如果
 count小于实际读到的字节，会内存溢出！不支持多线程
 同时读，否则出现不可预料的错误。
**********************************************/
int usb_read(struct ys_usb_handle *handle, unsigned char *data, int count)
{
	unsigned char *rxbuf = NULL;
	int ret,headersize,packetsize,offset,actual_length,size;
	struct usb_driver_header header;
#if CRC_ENABLE
	unsigned char crc;
#endif

//	pthread_mutex_lock(&(handle->rwlock));

	headersize = sizeof(struct usb_driver_header);

	if (count >= MAX_BULK_LENGTH)
		packetsize = MAX_BULK_LENGTH;
	else
		packetsize = count;

	rxbuf = malloc(packetsize+headersize);
	if (rxbuf == NULL) {
		ysprint("usb_read() malloc buffer failed!\n");
		return -1;
	}

	size          = 0;
	offset        = 0;
	actual_length = 0;

	while(1)
	{
		memset(rxbuf, 0x0, packetsize+ headersize);
		ret = libusb_bulk_transfer(handle->libusb_handle, handle->Endpoint_in, rxbuf, packetsize+headersize, &actual_length, 0);
		if (ret < 0) {
			if (ret != -7)
				ysprint("USB read data failed,ret = %d,USB idProduct = %x\n", ret, handle->idproduct);
			break;
		}
		else 
		{
			header.num    = rxbuf[0];
			header.length = rxbuf[1];
			header.type   = rxbuf[2];
			header.crc    = rxbuf[3];
			header.size   = *(unsigned int *)(rxbuf+4);

			if (actual_length- header.size == 1) {
				//ysprint("why here!!!!!!!!!!, %d- %d\n", actual_length, header.size);
				actual_length = header.size;
			}

			if (header.num < header.length) 
			{
				size += actual_length- headersize;

				if(offset+ actual_length- headersize > count) {
					ysprint("usb_read out of memory!\n");
					ret = OUTOFMEM;
					break;
				}

				memcpy(data+ offset,rxbuf+ headersize,actual_length- headersize);
#if CRC_ENABLE
				crc = get_crc(rxbuf+ headersize, actual_length- headersize);
				if (header.crc != crc) {
					ysprint("%s: %d %s: CRC error: subframe %d, nframes %d, header.size %d, header.crc %d, get crc = %d.\n",
							__FILE__, __LINE__, __FUNCTION__, header.num- 1, header.length, header.size, header.crc, crc);
					ret = CRCERROR;
					break;
				}
#endif

				offset += actual_length-headersize;
			}
			else if (header.num == header.length)
			{
				size += actual_length-headersize;

				if (offset+ actual_length-headersize > count) {
					ysprint("usb_read out of memory!\n");
					ret = OUTOFMEM;
					break;
				}

				memcpy(data+offset,rxbuf+headersize,actual_length-headersize);
#if CRC_ENABLE
				crc = get_crc(rxbuf+ headersize, actual_length-headersize);
				if (header.crc != crc) {
					ysprint("%s: %d %s: CRC error: subframe %d, nframes %d, header.size %d, header.crc %d, get crc = %d.\n",
							__FILE__, __LINE__, __FUNCTION__, header.num- 1, header.length, header.size, header.crc, crc);
					ret = CRCERROR;
					break;
				}
#endif
				offset += actual_length-headersize;
				break;
			}
			else
			{
				ysprint("usb_read the data error,header.num = %d, header.length = %d\n",header.num, header.length);
				ret = SEQERROR;
				break;
			}
		}
	}

//	pthread_mutex_unlock(&(handle->rwlock));

	if (ret == 0)
		ret = size;

	if (rxbuf) {
		free(rxbuf);
		rxbuf = NULL;
	}

	return ret;
}

/**********************************************
 函数说明：将程序加载到dsp运行。成功返回0,返回其他
 值都是错误的！
**********************************************/
int load_dsp(struct ys_usb_handle *handle,const char *filepath)
{
    FILE *fp;
    int actual_length;
    unsigned int FileSize,ret;
    unsigned int headersize,num,count,length;
    unsigned int SizeperSend = 400;
    unsigned int *head_ptr;
    unsigned char *data,rxbuf[1];
    unsigned int magic;
#if 0 
    unsigned char *temp,image_crc;
    int k;
#endif
    
    fp=fopen(filepath,"r");
    if(fp==NULL){
        ysprint("dsp image %s open failed!",filepath);
        return(-20);
    }
        
    fread(&magic,1,4,fp);
    if(magic!=0x43525052){
        ysprint("File type is wrong! Filepath:%s,magic = %x\n",filepath,magic);
        fclose(fp);
        sleep(1);
        return(-10);
    }

    // goto start of file
    fseek(fp,0,SEEK_SET);
        
    // read file size
    fseek(fp,0,SEEK_END);
    FileSize = ftell(fp);
    
#if 0
    ysprint("FileSize = %d\n",FileSize);
    // goto start of file
    fseek(fp,0,SEEK_SET);
    temp = (unsigned char *)malloc(FileSize);
    fread(temp,1,FileSize,fp);
    image_crc = get_crc(temp,FileSize);
    ysprint("image_crc = %d\n",image_crc);
    
    ysprint("image begin\n");
    for(k=0;k<FileSize;k++){
        ysprint("%d\n",temp[k]);
    }
    ysprint("image end\n");
    free(temp);
    temp = NULL;
#endif
       
    length = FileSize%SizeperSend;
    if(length==0) length = FileSize/SizeperSend;
    else length = FileSize/SizeperSend + 1;
    
    // goto start of file
    fseek(fp,0,SEEK_SET);
    headersize = sizeof(struct usb_app_header);
    data = (unsigned char *)malloc(SizeperSend+headersize);
    if(data==NULL)
    {
        //ysprint();
        return -1;
    }
    head_ptr = (unsigned int *)data;
    
    num = 0;
    count = 0;
    
    while(1)
    {
        num++;
        memset(data,0,SizeperSend+headersize);
        
        head_ptr[0] = APP_IDENTIFY;
        head_ptr[1] = num;
        head_ptr[2] = length;
        head_ptr[3] = FileSize;
        
        if(FileSize-count>SizeperSend)
        {
            fread(data+headersize,1,SizeperSend,fp);
            head_ptr[4] = get_crc(data+headersize,SizeperSend);
            usb_write(handle,data,SizeperSend+headersize);
            count += SizeperSend;
        }
        else
        {
            fread(data+headersize,1,FileSize-count,fp);
            head_ptr[4] = get_crc(data+headersize,FileSize-count);
            usb_write(handle,data,FileSize-count+headersize);
            break;
        } 
    } 
    free(data);
    data = NULL;
    
    fclose(fp);
    
    //等待dsp返回
	ret = libusb_bulk_transfer(handle->libusb_handle,handle->Endpoint_in,rxbuf,sizeof(rxbuf),&actual_length,100);
	if(ret<0){
		return ret;
	}else{
		return rxbuf[0];
	}
}

/**********************************************
 函数说明：更新ubl函数。成功返回0,返回其他
 值都是错误的！
**********************************************/
int update_ubl(struct ys_usb_handle *handle,const char *filepath)
{
    FILE *fp;
    int actual_length;
    unsigned int FileSize,ret;
    unsigned int headersize,num,count,length;
    unsigned int SizeperSend = 400;
    unsigned int *head_ptr;
    unsigned char *data,rxbuf[1];
    unsigned int magic;
#if 0 
    unsigned char *temp,image_crc;
    int k;
#endif
    
    fp=fopen(filepath,"r");
    if(fp==NULL){
        ysprint("%s open failed!",filepath);
        return(-20);
    }
        
    fread(&magic,1,4,fp);
    if(magic!=0x41504954){
        ysprint("File type is wrong! Filepath:%s,magic = %x\n",filepath,magic);
        fclose(fp);
        sleep(1);
        return(-10);
    }

    // goto start of file
    fseek(fp,0,SEEK_SET);
        
    // read file size
    fseek(fp,0,SEEK_END);
    FileSize = ftell(fp);
    
    /*if(FileSize>65535){
        ysprint("FILE:%s is too large!\n",filepath);
        return -5;
    }*/
    
#if 0
    ysprint("FileSize = %d\n",FileSize);
    // goto start of file
    fseek(fp,0,SEEK_SET);
    temp = (unsigned char *)malloc(FileSize);
    fread(temp,1,FileSize,fp);
    image_crc = get_crc(temp,FileSize);
    ysprint("image_crc = %d\n",image_crc);
    
    ysprint("image begin\n");
    for(k=0;k<FileSize;k++){
        ysprint("%d\n",temp[k]);
    }
    ysprint("image end\n");
    free(temp);
    temp = NULL;
#endif
       
    length = FileSize%SizeperSend;
    if(length==0) length = FileSize/SizeperSend;
    else length = FileSize/SizeperSend + 1;
    
    // goto start of file
    fseek(fp,0,SEEK_SET);
    headersize = sizeof(struct usb_app_header);
    data = (unsigned char *)malloc(SizeperSend+headersize);
    if(data==NULL){
        //ysprint("");
        return -1;
    }
    head_ptr = (unsigned int *)data;
    
    num = 0;
    count = 0;
    
    while(1)
    {
        num++;
        memset(data,0,SizeperSend+headersize);
        
        head_ptr[0] = CMD_UPDATE_UBL;
        head_ptr[1] = num;
        head_ptr[2] = length;
        head_ptr[3] = FileSize;
        
        if(FileSize-count>SizeperSend)
        {
            fread(data+headersize,1,SizeperSend,fp);
            head_ptr[4] = get_crc(data+headersize,SizeperSend);
            usb_write(handle,data,SizeperSend+headersize);
            count += SizeperSend;
        }
        else
        {
            fread(data+headersize,1,FileSize-count,fp);
            head_ptr[4] = get_crc(data+headersize,FileSize-count);
            usb_write(handle,data,FileSize-count+headersize);
            break;
        } 
    } 
    free(data);
    data = NULL;
    
    fclose(fp);
    
    //等待dsp返回
	ret = libusb_bulk_transfer(handle->libusb_handle,handle->Endpoint_in,rxbuf,sizeof(rxbuf),&actual_length,100);
	if(ret<0){
		return ret;
	}else{
		return rxbuf[0];
	}
}

/**********************************************
 函数说明：通过usb读取c674x的EEPROM。成功返回读到
 的字节数，失败返回负数。
**********************************************/
int usb_read_i2c(struct ys_usb_handle *handle,unsigned char *data,unsigned char addr,unsigned char count)
{
    int actual_length,ret;
    unsigned int headersize;
    unsigned int *head_ptr;
    unsigned char *buffer,rxbuf[128];
    
    headersize = sizeof(struct usb_app_header);
    buffer = (unsigned char *)malloc(count+headersize+1);
    if(buffer==NULL){
        //ysprint();
        return -1;
    }
    memset(buffer,0,count+headersize+1);
    head_ptr = (unsigned int *)buffer;
        
    head_ptr[0] = CMD_READ_I2C;
    head_ptr[1] = 0;
    head_ptr[2] = 0;
    head_ptr[3] = 0;    
    
    buffer[headersize] = count;
    buffer[headersize+1] = addr;
    
    usb_write(handle,buffer,count+headersize+1);  
    
    free(buffer);
    buffer = NULL;
    
    //等待dsp返回
	ret = libusb_bulk_transfer(handle->libusb_handle,handle->Endpoint_in,rxbuf,sizeof(rxbuf),&actual_length,0);
	if(ret<0){
		return ret;
	}else{
		memcpy(data,rxbuf,count);
		usleep(20000);//延时20毫秒再返回，这是因为频繁调用读写函数会出问题！
		return actual_length;
	}    
}

/**********************************************
 函数说明：通过usb写c674x的EEPROM。成功返回0,返回其他
 值都是错误的！
**********************************************/
int usb_write_i2c(struct ys_usb_handle *handle,const unsigned char *data,unsigned char addr,unsigned char count)
{
    int actual_length,ret;
    unsigned int headersize;
    unsigned int *head_ptr;
    unsigned char *buffer,rxbuf[1];
    
    headersize = sizeof(struct usb_app_header);
    buffer = (unsigned char *)malloc(count+headersize+3);
    if(buffer==NULL){
        //ysprint();
        return -1;
    }
    memset(buffer,0,count+headersize+3);
    head_ptr = (unsigned int *)buffer;
        
    head_ptr[0] = CMD_WRITE_I2C;
    head_ptr[1] = 0;
    head_ptr[2] = 0;
    head_ptr[3] = 0;    
    
    buffer[headersize] = count;
    buffer[headersize+1] = addr;
    
    memcpy(buffer+headersize+2,data,count);
    usb_write(handle,buffer,count+headersize+3);  
    
    free(buffer);
    buffer = NULL;
    
    //等待dsp返回
	ret = libusb_bulk_transfer(handle->libusb_handle,handle->Endpoint_in,rxbuf,sizeof(rxbuf),&actual_length,0);
	if(ret<0){
		return ret;
	}else{
		usleep(20000); //延时20毫秒再返回，这是因为频繁调用读写函数会出问题！
		return rxbuf[0];
	}
}

/**********************************************
 函数说明：初始化usb看门狗
**********************************************/
int usb_watchdog_start(struct ys_usb_handle *handle,unsigned char flag)
{
    unsigned int headersize;
    unsigned int *head_ptr;
    unsigned char *buffer;
    
    headersize = sizeof(struct usb_app_header);
    buffer = (unsigned char *)malloc(headersize);
    if(buffer==NULL){
        //ysprint();
        return -1;
    }
    memset(buffer,0,headersize);
    head_ptr = (unsigned int *)buffer;
        
    if(flag==1){
        head_ptr[0] = CMD_WATCHDOG_START;
        head_ptr[1] = 0;
        head_ptr[2] = 0;
        head_ptr[3] = 0;  
    }else if(flag==2){
        head_ptr[0] = CMD_HEADER1;
        head_ptr[1] = CMD_HEADER2;
        head_ptr[2] = CMD_WATCHDOG_START;
        head_ptr[3] = 0;        
    }  
    
    usb_write(handle,buffer,headersize);  
    
    free(buffer);
    buffer = NULL;    
    
    return 0;
}

/**********************************************
 函数说明：产生一个随机数，内部函数，不对外开放！
**********************************************/
static void get_random_bytes(void* buf, int nbytes)
{
	int i;
	int fd = -1;
	int lose_counter = 0;
	char *cp = (char*)buf;
	struct timeval tv;
	static unsigned seed = 0;
	if (fd > -1){
		while (nbytes > 0){
			i = read (fd, cp, nbytes);
			if ((i < 0) && ((errno == EINTR) || (errno == EAGAIN)))
				continue;
			if (i <= 0){
				if (lose_counter++ == 8)
					break;
				continue;
			}
			nbytes -= i;
			cp += i;
			lose_counter = 0;
		}
		close(fd);
		if(lose_counter < 8)
			return;
	}
	for (i = 0; i < nbytes; i++)
	{
		if (seed == 0){
			gettimeofday(&tv, 0);
			seed = (getpid() << 13) ^ tv.tv_usec ^ getuid() ^ tv.tv_sec;
		}
		*cp++ = rand_r(&seed) & 0xFF;
	}
	return;
}

/**********************************************
 函数说明：dm2016校验，成功返回0,失败返回负数。应先
 调用这个函数，成功了再传输dsp app。
**********************************************/
#if 0
int dm2016_verify(struct ys_usb_handle *handle)
{
	int res,i,ret,actual_length;
	unsigned char data[8],rxdata[8];
	unsigned char decdata[8];
    unsigned int headersize;
    unsigned int *head_ptr;
    unsigned char *buffer;

	get_random_bytes(data, 8);
	for(i = 0; i < 8; i ++)
		decdata[i] = data[i];    

	res = key_verify(decdata,0);
	if(res < 0)
		return -1;

    headersize = sizeof(struct usb_app_header);
    buffer = (unsigned char *)malloc(headersize+8);
    if(buffer==NULL){
        //ysprint();
        return -1;
    }
    memset(buffer,0,headersize+8);
    head_ptr = (unsigned int *)buffer;
        
    head_ptr[0] = CMD_DM2016_VERIFY;
    head_ptr[1] = 0;
    head_ptr[2] = 0;
    head_ptr[3] = 0;    
    
    memcpy(buffer+headersize,decdata,8);
    usb_write(handle,buffer,headersize+8);  
    
    free(buffer);
    buffer = NULL;
    
    //等待dsp返回
	ret = libusb_bulk_transfer(handle->libusb_handle,handle->Endpoint_in,rxdata,sizeof(rxdata),&actual_length,5000);
	if(ret<0){
		return ret;
	}else{
		if(actual_length!=8) return -99;
		if(strncmp((const char *)rxdata,(const char *)data,8)==0) return 0;
		else return -98;
	}
}
#endif
/**********************************************
 函数说明：判断当前程序处于bootloader还是app!返回
 1表示当前在bootloader,返回2表示当前在app。
**********************************************/
int At_UBLorAPP(struct ys_usb_handle *handle)
{
    int ret,actual_length;
    unsigned int headersize;
    unsigned int *head_ptr;
    unsigned char *buffer,rxdata[1];
    
    headersize = sizeof(struct usb_app_header);
    buffer = (unsigned char *)malloc(headersize);
    if(buffer==NULL){
        //ysprint();
        return -1;
    }
    memset(buffer,0,headersize);
    head_ptr = (unsigned int *)buffer;
        
    head_ptr[0] = CMD_ATUBLORNOT;
    head_ptr[1] = 0;
    head_ptr[2] = 0;
    head_ptr[3] = 0;  
    
    usb_write(handle,buffer,headersize);  
    
    free(buffer);
    buffer = NULL;    

    //等待dsp返回
	ret = libusb_bulk_transfer(handle->libusb_handle,handle->Endpoint_in,rxdata,sizeof(rxdata),&actual_length,100);
	if(ret<0){
		return 2;
	}else{
		if(rxdata[0]==10) return 1;
		else return 0;
	}   
}
/**********************************************
 函数说明：USB关闭函数，释放USB资源。
**********************************************/
int usb_close(struct ys_usb_handle *handle)
{
	//pthread_mutex_destroy(&(handle->lock));
	libusb_release_interface(handle->libusb_handle, 0);
	libusb_close(handle->libusb_handle);
	//libusb_exit(NULL);
	//ysprint("555555555555555555\n");
	return 0;
}

