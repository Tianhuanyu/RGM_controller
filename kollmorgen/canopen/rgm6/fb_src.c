#include "fb.src.h"



const char delims_1[] = ";";
const char delims_2[] = ",";

int fb_tcp_queue(char* temp_buff){

    // const sub_receivebuf_long
    char* result = NULL;
    char* sub_result = NULL;
    int num = 0;
    char sub_buff[100];
    // char sub_buff_v[100];
    // char sub_buff_tm[100];
    char sub_buff_r[100];
    char *key,*keyp,*keyv,*keyt,*keys;
    
    /*insert a queue*/
    
    //eprintf("run to here1 %s\n",sub_receivebuf);
    strcpy(sub_buff,temp_buff);
    result = strtok_r(sub_buff,delims_1,&key);
    while(result != NULL){
        switch(*result){
            //num = 0;
            case 'F':
                    num = 0;
                    strcpy(sub_buff_r,result);
                    printf("sub_buff = %s\n",sub_buff_r);

                    sub_result = strtok_r(sub_buff_r,delims_2,&keyp);
                    while((sub_result != NULL)&&(num<7)){
                        if(num ==0){
                            sub_result = substring(sub_result,2,9);
                        }
                        if(num<3)
                            pRGM->target_tcp_frame.point[num] = strtod(sub_result,NULL);
                        else
                            pRGM->target_tcp_frame.orientation[num-3] = strtod(sub_result,NULL);

                        sub_result = strtok_r(NULL,delims_2,&keyp);
                        num = num+1 ;
                    }
                
                break;
            case 'T':
              
                break;
            case 'W':

                break;

            default :
                return 1;
            
        }
        result = strtok_r(NULL,delims_1,&key);
    }
    

    return 0;
    
}