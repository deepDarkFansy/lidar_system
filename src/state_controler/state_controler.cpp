#include "node_c/state_controler.h"


#define PORT 8888

State::State(ros::NodeHandle &node){
    this->node = node;
    initPublisher();
    state = ST_WAIT;
    run();
}


void State::initPublisher(){
    pub = node.advertise<std_msgs::Int32>(
        "/state", 100
    );
}


void State::initSubscriber(){
    sub = node.subscribe<std_msgs::Int32>(
        "/command", 100, &State::changeStateCbk, this
    );
}


void State::changeStateCbk(const std_msgs::Int32ConstPtr &msgs){
    if(state_my (msgs->data) == ST_STOP){
        state = ST_STOP;
    }
}

void State::run(){

    int lfd;
    struct sockaddr_in serv_addr, clit_addr;
    socklen_t clit_len;

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if((lfd = socket(AF_INET, SOCK_STREAM, 0)) == -1){
        std::cout<<"create error"<<std::endl;
    }
    if((bind(lfd, (sockaddr *) &serv_addr, sizeof(serv_addr)))==-1){
        std::cout<<"bind error"<<std::endl;
    }

    listen(lfd, 128);

    clit_len = sizeof(clit_addr);

    int cfd;

    char buf[4000];
    int ret;

    std::cout<<"Waiting for client's request"<<std::endl;

    while(true){
        if((cfd = accept(lfd, (sockaddr *) &clit_addr, &clit_len))==-1){
            std::cout<<"acc error"<<std::endl;
        }else{
            strcpy(buf, "Connect succeed! Please input you command:\n");
            write(cfd, buf, strlen(buf));
        }
        while(true){
            
            ret = read(cfd, buf, sizeof(buf));

            for(int i=0; i<ret; i++){
                buf[i] = toupper(buf[i]);
            }
            buf[ret] ='\0';
            if(strcmp(buf, "QUIT\n") == 0){
                break;
            }

            std_msgs::Int32 flag;

            switch(state){
                case ST_WAIT:
                    if(strcmp(buf, "START\n") == 0){
                        state = ST_ACTIVE;
                    }else{
                        strcpy(buf, "Wrong command! Please input the correct command(START,QUIT):\n");
                        ret = strlen(buf);
                    }
                    break;
                case ST_ACTIVE:
                    if(strcmp(buf, "STOP\n") == 0){
                        state = ST_STOP;
                    }else{
                        strcpy(buf, "Wrong command! Please input the correct command(STOP, QUIT):\n");
                        ret = strlen(buf);
                    }
                    break;
                case ST_STOP:
                    if(strcmp(buf, "WAIT\n") == 0){
                        state = ST_WAIT;
                    }else if(strcmp(buf, "RESTART\n") == 0){
                        state = ST_ACTIVE;
                    }else{
                        strcpy(buf, "Wrong command! Please input the correct command(RESTART, WAIT, QUIT):\n");
                        ret = strlen(buf);
                    }
                    break;
                default:
                    break;
            }

            flag.data = state;
            pub.publish(flag);

            write(cfd, buf, ret);
            
        }
        close(cfd);
    }
    close(lfd);
}