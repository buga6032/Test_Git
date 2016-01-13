#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <termio.h>
#include <time.h>
#include <pthread.h>

#define JIMMY_GPIO  1
#define N_SIZE 0x03
#define TRUE  1
#define FALSE 0
#define GPIO_NAME  "/dev/c6742_reset"

static char *dev  = "/dev/ttymxc3"; 
static struct termio oterm_attr;

unsigned char opcode[3] = {0x59,0x53,0x58};
unsigned char magic[4] = {0x54,0x49,0x50,0x41};
unsigned char func_exec_cmd[4] = {0x0d,0x59,0x53,0x58};
unsigned char sec_load_cmd[4] = {0x01,0x59,0x53,0x58};
unsigned char jump_close_cmd[4] = {0x06,0x59,0x53,0x58};
unsigned char recv_func_exec_cmd[4] = {0x0d,0x59,0x53,0x52};
unsigned char recv_sec_load_cmd[4] = {0x01,0x59,0x53,0x52};
unsigned char recv_jump_close_cmd[4] = {0x06,0x59,0x53,0x52};

unsigned char xmt_start[1]={0x58},recv_start[1]={0x52};
unsigned char ping_device[4] = {0x0b,0x59,0x53,0x58};
unsigned char recv_ping_device[4] = {0x0b,0x59,0x53,0x52};
unsigned char temp[4] = {0x0d,0x59,0x53,0x58};
unsigned char recv_temp[4] = {0x0d,0x59,0x53,0x52};

unsigned char N_LEN[4] = {N_SIZE,0x00,0x00,0x00};
unsigned char N_DAT[N_SIZE][4] = {
    {0x01,0x00,0x00,0x00},
    {0x02,0x00,0x00,0x00},
    {0x03,0x00,0x00,0x00}
};

static int baudflag_arr[] = {
    B921600, B460800, B230400, B115200, B57600, B38400,
    B19200, B9600, B4800, B2400, B1800, B1200,
    B600, B300, B150, B110, B75, B50
};

static int speed_arr[] = {
    921600, 460800, 230400, 115200, 57600, 38400,
    19200, 9600, 4800, 2400, 1800, 1200,
    600, 300, 150, 110, 75, 50
};

/*函数声明*/
int setup_port(int fd, int baud, int databits, int parity, int stopbits);
int uart_write(int fd, void *buf, int len);
int uart_read(int fd, void *buf, int len);
int speed_to_flag(int speed);
int reset_port(int fd);

int sendais(unsigned char *buffer,FILE *fp,int fd);

static unsigned char dsp_reset_flag = 0;

void *reset_dsp(void *arg)
{	
    char value;
    int dev_fd;
    
    dev_fd = *(int *)arg;
    
    while(dsp_reset_flag==0){
        usleep(10000);
    }
    
#if JIMMY_GPIO                                           //dsp复位 
    value = 0;
    write(dev_fd,&value,1);
    usleep(30000);
    value = 1;
    write(dev_fd,&value,1);
    close(dev_fd);
    
    //printf("dsp reset complete!\n");
#endif
}

int main(int argc,char **argv)
{
    FILE *fp;
    int fd,ret,filesize,i;
    unsigned char buf[20],*data;
    
    
    if(argc!=2){
        printf("Line:%d,argc error!!!\n",__LINE__);
        return -1;
    }
    
    ret = access(argv[1],F_OK);
    if(ret!=0){
        printf("Line:%d,%s not exist!!!\n",__LINE__,argv[1]);
        return -1;
    }
    
#if JIMMY_GPIO
	int dev_fd;
	pthread_t tid;
	
	dev_fd = open(GPIO_NAME,O_RDWR | O_NONBLOCK);
	if ( dev_fd == -1 ) {
		printf("Line:%d,open %s failed!!!\n",__LINE__,GPIO_NAME);
		return -1;
	}
	
	ret = pthread_create(&tid,NULL,reset_dsp,(void *)&dev_fd);
    if(ret){
	    printf("creat read thread fail\n");
	    close(dev_fd);
	    return -1;
    }
#endif
    
    fd = open(dev,O_RDWR);                               //打开串口  | O_NOCTTY | O_NDELAY
    if(fd<0)   
    {                  
        printf("Can't Open Serial Port\n");
        return -1;       
    }else{
        printf("%s open successful!\n",dev);
    }   

    ret = setup_port(fd,115200,8,0,1);                   //配置串口，波特率，数据位，奇偶校验位，停止位。
    if(ret!=0){
        printf("setup port failed!\n");
    }else{
        printf("setup port successful!\n");
    }
    
    // while(1){
    //     uart_write(fd,"11111111",8);
    //    usleep(10000);
    // }
    
    //wait for bootme
    printf("Wait for bootme...\n");
    dsp_reset_flag = 1;                                  //DSP复位
    while(1)
    {
        memset(buf,0x00,sizeof(buf));
        uart_read(fd,buf,10);
        
        //可能会读到几个乱码，要跳过
        for(i=0;i<sizeof(buf);i++){
            if(buf[i]>='A'&&buf[i]<='Z')
                break;    
        }
        
        if(strncmp(buf+i,"BOOTME",6)!=0){
            printf("buffer:%s\n",buf);
        }else{
            break;
        }
    }
    
head:
    //xmt_start
    printf("Performing start word sync...\n");
    while(1)
    {
        uart_write(fd,xmt_start,1);
        uart_read(fd,buf,1);
        if(buf[0]==recv_start[0]) break;
        else {
        //    printf("xmt start failed!\n");
        }
    }
    
    //ping_device
    printf("Performing ping opcode sync...\n");
    while(1)
    {
        uart_write(fd,ping_device,4);
        uart_read(fd,buf,4);
        if(strncmp(buf,recv_ping_device,4)==0) break;
        else {
        //    printf("ping device failed!\n");
        }
    }
    
    //send N
    uart_write(fd,N_LEN,4);
    uart_read(fd,buf,4);
    if(strncmp(buf,N_LEN,4)!=0) {
        printf("send N length failed,goto head.\n");
        goto head;
    }
    for(i=0;i<N_SIZE;i++)
    {
        uart_write(fd,N_DAT[i],4);
        uart_read(fd,buf,4);
        if(strncmp(buf,N_DAT[i],4)!=0) {
            printf("send N sequence failed,goto head.\n");
            goto head;
        }
    }
    
    printf("Handshake successful,send code...\n");
    

    fp = fopen(argv[1],"r");
    if(fp==NULL){
        printf("Line:%d,file open failed!\n",__LINE__);
        return -1;
    }
        
    // read file size
    fseek(fp,0,SEEK_END);
    filesize = ftell(fp);
    
    data = (unsigned char *)malloc(filesize);
    memset(data,0x00,filesize);
    fseek(fp,0,SEEK_SET);
    fread(data,1,filesize,fp);
    fclose(fp);
    
    if(strncmp(data,magic,4)!=0)
    {
        printf("Line:%d,file format error!!!\n",__LINE__);
        free(data);
        data = NULL;
        return -1;
    }
    
    sleep(1);                          //延时一秒，让dsp进行以上命令的处理。
    
    sendais(data,fp,fd);               //发送ais文件给dsp
    
    uart_read(fd,buf,8);
    for(i=0;i<8;i++){
        if(buf[i]>'A'&&buf[i]<'Z')
            break;
    }
    if(strncmp(buf+i,"DONE",4)==0){
        printf("uart boot successful!\n");
    }
    else{
        printf("uart boot failed!\n");
        return 0;
    }

    free(data);
    data = NULL;
#if 0    
    while(1)                           //循环读取dsp发送过来的消息，打印到arm端控制台。
    {
        ret = uart_read(fd,buf,19);
        if(ret==0) continue;
        
        printf("%s\n",buf);
    }
#endif    
    reset_port(fd);
    close(fd);

    //usb_test();

    return 0;
}

int sendais(unsigned char *buffer,FILE *fp,int fd)
{
    unsigned int offset=4,size,data;
    unsigned char buf[4];
    short count;
    int i,ret;
    
   while(1)
    {        
        if(strncmp(buffer+offset+1,opcode,3)==0)
        {
            if(strncmp(buffer+offset,func_exec_cmd,4)==0)
            {    
func_exec_cmd:                   
                uart_write(fd,buffer+offset,4);
                offset += 4;
                uart_read(fd,buf,4);
                if(strncmp(buf,recv_func_exec_cmd,4)!=0){
                    printf("func exec cmd sync failed!\n");
                    offset -= 4;
                    sleep(1);
                    goto func_exec_cmd;
                    //return -1;
                }
                else{
                    //fseek(fp,offset+2,SEEK_SET);
                    //fread(&count,2,1,fp);         //获取该命令的参数个数
                    
                    count = *(short *)(buffer+offset+2);
                    uart_write(fd,buffer+offset,4);
                    offset += 4;                    
                    for(i=0;i<count;i++)
                    {
                        uart_write(fd,buffer+offset,4);
                        offset += 4;
                    } 
                }    
                usleep(20000);                      //这里必须延时，这个命令过去dsp的RBL会去调用相关的函数，需要时间！                 
            }
            else if(strncmp(buffer+offset,jump_close_cmd,4)==0)
            { 
jump_close_cmd:            
                uart_write(fd,buffer+offset,4);
                offset += 4;
                uart_read(fd,buf,4);
                if(strncmp(buf,recv_jump_close_cmd,4)!=0){
                    printf("jump close cmd sync failed!\n");
                    offset -= 4;
                    sleep(1);
                    goto jump_close_cmd;
                    //return -1;
                }
                else{
                    uart_write(fd,buffer+offset,4);
                    offset += 4;
                    break;
                }                 
            }
            else if(strncmp(buffer+offset,sec_load_cmd,4)==0)
            {
sec_load_cmd:            
                ret = uart_write(fd,buffer+offset,4);            //写命令
                if(ret==-1) {
                    printf("uart write failed!!!\n");
                    return -1;
                }
                offset += 4;                                   
                
                uart_read(fd,buf,4);
                if(strncmp(buf,recv_sec_load_cmd,4)!=0){
                    printf("sec load cmd sync failed!\n");
                    offset -= 4;
                    sleep(1);
                    goto sec_load_cmd;                
                    //return -1;
                }
                else{                
                    uart_write(fd,buffer+offset,4);             //写参数地址
                    offset += 4;
                    size = *(unsigned int *)(buffer+offset);
                    
                    uart_write(fd,buffer+offset,4);             //写参数个数
                    offset += 4;
                    if(size%4==0){
                        for(i=0;i<size/4;i++)
                        {
                            uart_write(fd,buffer+offset,4);
                            offset += 4;
                        }
                    }else{
                        count = size/4;
                        for(i=0;i<count/4;i++)
                        {
                            uart_write(fd,buffer+offset,4);
                            offset += 4;
                        }     
                        count = size%4;
                        fseek(fp,offset,SEEK_SET);
                        fread(&data,count,1,fp);
                        uart_write(fd,&data,4);                  
                    }
                }              
            }
            else
            {
                printf("cmd is invalid!!!\n");
                return -1;
            }
        }
        else
        {
            printf("opcode is error,offset = %d\n",offset);
            return -1;
        }
    }    
    
    return 0;
}


/******************************************************************************
 * NAME:
 * speed_to_flag
 *
 * DESCRIPTION:
 * Translate baudrate into flag
 *
 * PARAMETERS:
 * speed - The baudrate to convert
 *
 * RETURN:
 * The flag
 ******************************************************************************/
int speed_to_flag(int speed)
{
    int i;

    for (i = 0; i < sizeof(speed_arr)/sizeof(int); i++) {
        if (speed == speed_arr[i]) {
            return baudflag_arr[i];
        }
    }

    fprintf(stderr, "Unsupported baudrate, use 9600 instead!\n");
    return B9600;
}

/******************************************************************************
 * NAME:
 * stup_port
 *
 * DESCRIPTION:
 * Set serial port (include baudrate, line control)
 *
 * PARAMETERS:
 * fd - The fd of serial port to setup
 * baud - Baudrate: 9600, ...
 * databits - Databits: 5, 6, 7, 8
 * parity - Parity: 0(None), 1(Odd), 2(Even)
 * stopbits - Stopbits: 1, 2
 *
 * RETURN:
 * 0 for OK; Others for ERROR
 ******************************************************************************/
int setup_port(int fd, int baud, int databits, int parity, int stopbits)
{
    struct termio term_attr;

    /* Get current setting */
    if (ioctl(fd, TCGETA, &term_attr) < 0) {
        return -1;
    }

    /* Backup old setting */
    memcpy(&oterm_attr, &term_attr, sizeof(struct termio));

    term_attr.c_iflag &= ~(INLCR | IGNCR | ICRNL | ISTRIP);
    term_attr.c_oflag &= ~(OPOST | ONLCR | OCRNL);
    term_attr.c_lflag &= ~(ISIG | ECHO | ICANON | NOFLSH);
    term_attr.c_cflag &= ~CBAUD;
    term_attr.c_cflag |= CREAD | speed_to_flag(baud);

    /* Set databits */
    term_attr.c_cflag &= ~(CSIZE);
    switch (databits) {
        case 5:
            term_attr.c_cflag |= CS5;
            break;

        case 6:
            term_attr.c_cflag |= CS6;
            break;

        case 7:
            term_attr.c_cflag |= CS7;
            break;

        case 8:
        default:
            term_attr.c_cflag |= CS8;
            break;
    }

    /* Set parity */
    switch (parity) {
        case 1: /* Odd parity */
            term_attr.c_cflag |= (PARENB | PARODD);
            break;

        case 2: /* Even parity */
            term_attr.c_cflag |= PARENB;
            term_attr.c_cflag &= ~(PARODD);
            break;

        case 0: /* None parity */
        default:
            term_attr.c_cflag &= ~(PARENB);
            break;
    }


    /* Set stopbits */
    switch (stopbits) {
        case 2: /* 2 stopbits */
            term_attr.c_cflag |= CSTOPB;
            break;

        case 1: /* 1 stopbits */
        default:
            term_attr.c_cflag &= ~CSTOPB;
            break;
    }

    term_attr.c_cc[VMIN] = 0;
    term_attr.c_cc[VTIME] = 50;

    if (ioctl(fd, TCSETAW, &term_attr) < 0) {
        return -1;
    }

    if (ioctl(fd, TCFLSH, 2) < 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************
 * NAME:
 * uart_read
 *
 * DESCRIPTION:
 * Read data from serial port
 *
 *
 * PARAMETERS:
 * fd - The fd of serial port to read
 * buf - The buffer to keep readed data
 * len - The max count to read
 *
 * RETURN:
 * Count of readed data
 ******************************************************************************/
int uart_read(int fd, void *buf, int len)
{
    int count;
    int ret;

    ret = 0;
    count = 0;

//    while (len > 0) {


    ret = read(fd, (char*)buf + count, len);
    if (ret < 1) {
        //fprintf(stderr, "Read error %s\n", strerror(errno));
        //break;

    }

    count += ret;
    len = len - ret;

//    }


    *((char*)buf + count) = 0;
    return count;
}


/******************************************************************************
 * NAME:
 * uart_write
 *
 * DESCRIPTION:
 * Write data to serial port
 *
 *
 * PARAMETERS:
 * fd - The fd of serial port to write
 * buf - The buffer to keep data
 * len - The count of data
 *
 * RETURN:
 * Count of data wrote
 ******************************************************************************/
int uart_write(int fd, void *buf, int len)
{
    int count;
    int ret;

    ret = 0;
    count = 0;

    while (len > 0) {

        ret = write(fd, (char*)buf + count, len);
        if (ret < 1) {
            fprintf(stderr, "Write error %s\n", strerror(errno));
           // break;
            return -1;
        }

        count += ret;
        len = len - ret;
    }

    return count;
}

/******************************************************************************
 * NAME:
 * reset_port
 *
 * DESCRIPTION:
 * Restore original setting of serial port
 *
 * PARAMETERS:
 * fd - The fd of the serial port
 *
 * RETURN:
 * 0 for OK; Others for ERROR
 ******************************************************************************/
int reset_port(int fd)
{
    if (ioctl(fd, TCSETAW, &oterm_attr) < 0) {
        return -1;
    }

    return 0;
}


