#include "node_c/tcp_info.h"

TcpInfo::TcpInfo(ros::NodeHandle &node){
    this->node =node;
    initPublisher();
    run();
}
   

void TcpInfo::initPublisher(){
    pub = node.advertise<std_msgs::Int32>(
        "/command", 100
    );
}

void TcpInfo::run(){
    int lfd;
    struct sockaddr_in serv_addr, clit_addr;
    socklen_t clit_len;

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(9527);
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

    if((cfd = accept(lfd, (sockaddr *) &clit_addr, &clit_len))==-1){
            std::cout<<"acc error"<<std::endl;
    }
    while(true){
        
        ret = read(cfd, buf, sizeof(buf));


        for(int i=0; i<ret; i++){
            buf[i] = toupper(buf[i]);
        }
        buf[ret] ='\0';
        std::cout<<buf;
        std_msgs::Int32 flag;
        if(strcmp(buf, "STOP\n") == 0)
        {
            flag.data = NO_STOP;
        }else if(strcmp(buf, "SLOW\n") == 0){
            flag.data = NO_SLOW;
        }else{
            flag.data = NO_NORMAL;
        }
        pub.publish(flag);

        write(cfd, buf, ret);
        
    }
    close(cfd);
    close(lfd);

}