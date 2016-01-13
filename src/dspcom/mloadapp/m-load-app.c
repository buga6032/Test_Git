#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/ioctl.h>
#include <pthread.h>

#include "ys_libusb.h"

#define  update_ubl_flag    "/persistent/ljm_ubl_update"
#define  update_ubl_path    "/persistent/ljm_bootloader.ais"

static struct ys_usb_handle ghandle[7];
static int portnum[7],handle_cnt,pos;


int main(int argc,char **argv)
{
    int ret;
    unsigned char buffer[10],i;
    unsigned char syscmd[30],start_flag=0;
    
    if(argc!=2){
        printf("argc error!!!\n");
        return -1;
    }
    memset(syscmd,0,sizeof(syscmd));
    
    ret = access(argv[1],F_OK);
    if(ret!=0){
        printf("File: %s not exist!!!\n",argv[1]);
        return -1;
    }
  
the_start: 
	handle_cnt = usb_open(ghandle,portnum);
	if(handle_cnt<=0){
		printf("not usb device found!\n");
		return -1;
	}else{
		printf("%d usb device open successful!\n",handle_cnt);
	}
   
    //arm每次启动，都去重启dsp
    if(start_flag)
    {
        start_flag = 0;
        
        for(i=0;i<handle_cnt;i++){
            pos = portnum[i];
            ret = At_UBLorAPP(&ghandle[pos]);
            usleep(50000);//必须延时！
            
            //printf("ljm debug , ret = %d\n",ret);
            usb_watchdog_start(&ghandle[pos],ret);
            
            printf("portnum:%d,dsp reboot...\n",pos);
        }
        
	    sleep(8);
	    goto the_start;
	}
	
#if 0 	
	for(i=0;i<handle_cnt;i++)
	{   
	    pos = portnum[i];
        printf("portnum:%d,dm2016 verify...\n",pos);
        ret = dm2016_verify(&ghandle[pos]);
        if(ret!=0){
            printf("portnum:%d,dm2016 verify failed!!!\n",pos);
            goto the_end;    
        }else{
            printf("portnum:%d,dm2016 verify successful.\n",pos);
        }
    }
#endif
    
    ret = access(update_ubl_flag,F_OK);//如果标记文件存在，就更新bootloader
    if(ret==0)
    {
        ret = access(update_ubl_path,F_OK);//判断要更新的bootloader是否存在，不存在程序就退出！
        if(ret==0)
        {
            for(i=0;i<handle_cnt;i++)
            {
                pos = portnum[i];
	            printf("portnum:%d,update bootloader...\n",pos);	
	            while(1){
                    ret = update_ubl(&ghandle[pos],update_ubl_path);
                    if(ret==0) break;
                    else {
                        sleep(1);
                        printf("portnum:%d,update ubl failed! try again...\n",pos);
                    }
                }
                printf("portnum:%d,update bootloader successful!\n",pos);
                 
                printf("portnum:%d,dsp reboot...\n",pos);
                usb_watchdog_start(&ghandle[pos],1);
            }
            memcpy(syscmd,"rm ",3);
            memcpy(syscmd+3,update_ubl_flag,sizeof(update_ubl_flag));
            system(syscmd);
            
            sleep(8);
            goto the_start;
        }
        else
        {
            printf("You are going to update bootloader,but the %s is not exist,load dsp failed!!!\n",update_ubl_path);
            goto the_end;
        }
    }
    
    for(i=0;i<handle_cnt;i++)
    {
        pos = portnum[i];
        
	    printf("portnum:%d,load dsp application(%s) start...\n",pos,argv[1]);
	    while(1){
            ret = load_dsp(&ghandle[pos],argv[1]);
            if(ret==0) break;
            else {
                sleep(1);
                printf("portnum:%d,load dsp error! try again...\n",pos);
            }
        }
        printf("portnum:%d,load dsp application complete!\n",pos);
    }

#if 0	// ljm eeprom and watchdog test

	usb_watchdog_start(&ghandle[0]);
	sleep(10);
	
    printf("usb i2c write ... \n");
    memset(buffer,6,10);
    ret = usb_write_i2c(&ghandle[0],buffer,0x00,10);
    if(ret==0) printf("usb i2c write successful\n");
    else {
        return -1;
        printf("usb i2c write failed,ret = %d\n",ret); 
    }
    
    printf("usb i2c read ...\n");
    ret = usb_read_i2c(&ghandle[0],buffer,0x00,10);
    if(ret>0){
        printf("i2c read success,ret = %d\n",ret);
        for(i=0;i<10;i++)
            printf("%x  ",buffer[i]);
        printf("\n");
    }else{
        printf("i2c read failed , ret = %d\n",ret);
        return -1;
    }
#endif

the_end:  
    printf("m-load-app exit...\n");
    for(i=0;i<handle_cnt;i++){
        //printf("hello i = %d\n",i);
	    pos = portnum[i];
	    usb_close(&ghandle[pos]);
	}
	libusb_exit(NULL);
	
	return 0;
}

