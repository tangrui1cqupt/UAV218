#include <iostream>
#include <queue>
#include <set>
#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <memory.h>
#include <string>
#include <fstream>
#include <mysql/mysql.h>
#include <mutex>
#include <stdlib.h>

using namespace std;

pthread_t server_accept_tid1, server_accept_tid2, server_accept_tid3, server_accept_tid4;
pthread_t server_control_tid;
pthread_t server_receive_tid1, server_receive_tid2, server_receive_tid3, server_receive_tid4;

int client_fd[4] = { -1, -1, -1, -1 };
std::string tablename[4] = { "UAV1_data", "UAV2_data", "UAV3_data", "UAV4_data" };
int save_flag = 0;
enum MSG { NONE, TAKE_OFF, HOLD, LAND, FOLLOW };

//定义地面站控制消息结构体
typedef struct Message_Ground_Control_Basic
{
    uint8_t message_ID;
    uint8_t control_cmd;
} Ground_Control_Message;

//定义地面站接收消息结构体
typedef struct Message_Ground_Receive_Basic
{
    double uav_x;
    double uav_y;
    double uav_z;
    double uav_vx;
    double uav_vy;
    double uav_vz;
    double roll;
    double pitch;
    double yaw;
    double eso_x;
    double eso_y;
    double eso_z;
    double eso_vx;
    double eso_vy;
    double eso_vz;
    double eso_dx;
    double eso_dy;
    double eso_dz;
} Ground_Receive_Message;

//MySQL connection pool
class MySQLPool
{
public:
    static MySQLPool& getInstance()
    {
        static MySQLPool instance;
        return instance;
    }

    MYSQL* getConnection()
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (!connections.empty()) {
            MYSQL* conn = connections.front();
            connections.pop();
            return conn;
        }
        return NULL;
    }

    void returnConnection(MYSQL* conn)
    {
        std::lock_guard<std::mutex> lock(mutex);
        connections.push(conn);
    }

    ~MySQLPool()
    {
        while (!connections.empty())
        {
            MYSQL* conn = connections.front();
            connections.pop();
            mysql_close(conn);
        }
    }

private:
    std::queue<MYSQL*> connections;
    std::mutex mutex;

    MySQLPool()
    {
        for (int i = 0; i < 5; ++i)
        {
            MYSQL* conn = mysql_init(NULL);
            if (conn && mysql_real_connect(conn, "localhost", "root", "528629", "Flight_data", 0, NULL, 0))
            {
                connections.push(conn);
            }
            else
            {
                std::cerr << "Error: Failed to connect to MySQL server" << std::endl;
                exit(1);
            }
        }
    }

    MySQLPool(const MySQLPool&) = delete;
    MySQLPool& operator=(const MySQLPool&) = delete;
};

//地面站作为TCP通信的服务端，四台无人机作为客户端
void* ground_accept(void* args)
{
    int sockfd;
    int i = *((int*)args);

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
    {
        printf("socket connect failed!\n");
        return 0;
    }

    struct sockaddr_in server_addr, client_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(8888 - i);

    int ret = bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (-1 == ret) {
        fprintf(stderr, "socket bind error, reason: %s\n", strerror(errno));
        return 0;
    }

    listen(sockfd, 10);

    socklen_t client_addr_len = sizeof(client_addr);
    client_fd[i] = accept(sockfd, (struct sockaddr*)&client_addr, &client_addr_len);
    printf("TCP连接已经建立——UAV%d\n", i+1);

    pthread_exit(0);
}

void* server_control(void* args)
{
    std::string command;
    Ground_Control_Message control_message;

    while (std::cin >> command)
    {
        if (command == "takeoff")
        {
            std::cout << "command: " << command << std::endl;
            control_message.message_ID = 1;
            control_message.control_cmd = TAKE_OFF;
            save_flag = 1;//写入数据库的标志位

            for (int i = 0; i < 4; i++)
            {
                if (client_fd[i] != -1)
                {
                    send(client_fd[i], &control_message, sizeof(control_message), 0);
                }
            }
        }
        else if (command == "land")
        {
            std::cout << "command: " << command << std::endl;
            control_message.message_ID = 3;
            control_message.control_cmd = LAND;

            for (int i = 0; i < 4; i++)
            {
                if (client_fd[i] != -1)
                {
                    send(client_fd[i], &control_message, sizeof(control_message), 0);
                }
            }
        }
        else if (command == "follow")
        {
            std::cout << "command: " << command << std::endl;
            control_message.message_ID = 4;
            control_message.control_cmd = FOLLOW;

            for (int i = 0; i < 4; i++)
            {
                if (client_fd[i] != -1)
                {
                    send(client_fd[i], &control_message, sizeof(control_message), 0);
                }
            }
        }
    }
    pthread_exit(0);
}

void* ground_receive(void* args)
{
    int i = *((int*)args);
    char buffer[1024];

    printf("开始接收第%d架无人机的数据\n", i+1);

    MySQLPool& pool = MySQLPool::getInstance();
    MYSQL* conn = pool.getConnection();
    if (!conn)
    {
        std::cerr << "Error: Failed to get MySQL connection" << std::endl;
        return 0;
    }

    std::string deleteQuery = "DELETE FROM " + tablename[i];
    if (mysql_query(conn, deleteQuery.c_str()))
    {
        std::cerr << "Error: " << mysql_error(conn) << std::endl;
    }

    while(1)
    {
        int len = recv(client_fd[i], buffer, sizeof(Ground_Receive_Message), 0);
        if(len == 0)
        {
            printf("transmission of data is closed\n");
            return 0;
        }
        else if(len < 0)
        {
            printf("transmission of data failed with error\n");
            return 0;
        }

        Ground_Receive_Message recv_uav_message;
        memset(&recv_uav_message, 0, sizeof(recv_uav_message));
        memcpy(&recv_uav_message, buffer, sizeof(recv_uav_message));
        if(save_flag == 1)
        {
            std::string query = "INSERT INTO " + tablename[i] + " (uav_x, uav_y, uav_z, uav_vx, uav_vy, uav_vz, roll, pitch, yaw, eso_x, eso_y, eso_z, eso_vx, eso_vy, eso_vz, eso_dx, eso_dy, eso_dz) VALUES (";
            query += std::to_string(recv_uav_message.uav_x) + ", ";
            query += std::to_string(recv_uav_message.uav_y) + ", ";
            query += std::to_string(recv_uav_message.uav_z) + ", ";
            query += std::to_string(recv_uav_message.uav_vx) + ", ";
            query += std::to_string(recv_uav_message.uav_vy) + ", ";
            query += std::to_string(recv_uav_message.uav_vz) + ", ";
            query += std::to_string(recv_uav_message.roll) + ", ";
            query += std::to_string(recv_uav_message.pitch) + ", ";
            query += std::to_string(recv_uav_message.yaw) + ", ";
            query += std::to_string(recv_uav_message.eso_x) + ", ";
            query += std::to_string(recv_uav_message.eso_y) + ", ";
            query += std::to_string(recv_uav_message.eso_z) + ", ";
            query += std::to_string(recv_uav_message.eso_vx) + ", ";
            query += std::to_string(recv_uav_message.eso_vy) + ", ";
            query += std::to_string(recv_uav_message.eso_vz) + ", ";
            query += std::to_string(recv_uav_message.eso_dx) + ", ";
            query += std::to_string(recv_uav_message.eso_dy) + ", ";
            query += std::to_string(recv_uav_message.eso_dz) + ")";

            if (mysql_query(conn, query.c_str()))
            {
                std::cerr << "Error: " << mysql_error(conn) << std::endl;
            }
        }
        usleep(100000);//10hz
    }
    pool.returnConnection(conn);
    pthread_exit(0);
}

int main()
{
    int uav_id[4] = {0,1,2,3}; //对应4架无人机
    pthread_create(&server_accept_tid1, NULL, ground_accept, &uav_id[0]); //开启监听线程
    pthread_create(&server_accept_tid2, NULL, ground_accept, &uav_id[1]);
    pthread_create(&server_accept_tid3, NULL, ground_accept, &uav_id[2]);
    pthread_create(&server_accept_tid4, NULL, ground_accept, &uav_id[3]);

    printf("listen...\n");

    while (! ( client_fd[0] != -1 && client_fd[1] != -1 && client_fd[2] != -1 && client_fd[3] != -1 ) )
    {
        sleep(1);
        cout<<"wait all uavs connection"<<endl;
    }//等待无人机全部连接
    cout<<"all uavs connection success"<<endl;

    pthread_create(&server_control_tid, NULL, server_control, NULL); //开启发送上位机命令的线程

    pthread_create(&server_receive_tid1, NULL, ground_receive, &uav_id[0]); //开启接收无人机消息的线程
    pthread_create(&server_receive_tid2, NULL, ground_receive, &uav_id[1]);
    pthread_create(&server_receive_tid3, NULL, ground_receive, &uav_id[2]);
    pthread_create(&server_receive_tid4, NULL, ground_receive, &uav_id[3]);

    while (1)
    {
        sleep(1);
    }
    return 0;
}



