
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/poll.h>
#include <unistd.h>

#define LOG_TAG "MessageQueue"
#include <utils/Log.h>
#include "CameraHal.h"
#include "MessageQueue.h"

//#define MLOGD   LOGD
#define MLOGD

static char gDisplayThreadCommands[][30] = {
		{"CMD_DISPLAY_PAUSE"},
        {"CMD_DISPLAY_START"},
        {"CMD_DISPLAY_STOP"},
		{"CMD_DISPLAY_FRAME"}
};
static char gPreviewThreadCommands[][30] = {
        {"CMD_PREVIEW_STAREQ"},
        {"CMD_PREVIEW_VIDEOSNAPSHOT"}
};
static char gSnapshotThreadCommands[][30] = {
        {"CMD_SNAPSHOT_SNAPSHOT"},
        {"CMD_SNAPSHOT_EXIT"}
};
static char gCommandThreadCommands[][30] = {
		// Comands
        {"CMD_PREVIEW_START"},
        {"CMD_PREVIEW_STOP"},
        {"CMD_PREVIEW_CAPTURE"},
        {"CMD_PREVIEW_CAPTURE_CANCEL"},
        {"CMD_PREVIEW_QBUF"},        
        
        {"CMD_AF_START"},
        {"CMD_AF_CANCEL"},
        
        {"CMD_EXIT"},
        
        // ACKs
        {"CMD_ACK"},
        {"CMD_NACK"}
 };
static char gInvalCommands[]={"CMD_UNKNOW"};
static char* MessageCmdConvert(char* msgQ, unsigned int cmd)
{    
    char *cmd_name = gInvalCommands;
    if (strcmp(msgQ,"displayCmdQ") == 0) {
        if (cmd < sizeof(gDisplayThreadCommands)/30) 
            cmd_name = (char*)gDisplayThreadCommands[cmd];
    } else if (strcmp(msgQ,"previewCmdQ") == 0) {
        if (cmd < sizeof(gPreviewThreadCommands)/30) 
            cmd_name = (char*)gPreviewThreadCommands[cmd];
    } else if (strcmp(msgQ,"commandCmdQ") == 0) {
        if (cmd < sizeof(gCommandThreadCommands)/30) 
            cmd_name = (char*)gCommandThreadCommands[cmd];
    } else if (strcmp(msgQ,"snapshotCmdQ") == 0) {
        if (cmd < sizeof(gSnapshotThreadCommands)/30) 
            cmd_name = (char*)gSnapshotThreadCommands[cmd];
    }
    return cmd_name;
}

MessageQueue::MessageQueue()
{
    int fds[2] = {-1,-1};
    
    pipe(fds);
    this->fd_read = fds[0];
    this->fd_write = fds[1];
    MsgQueName[0] = 0;
    strcat(MsgQueName, "CamMsgQue");
}
MessageQueue::MessageQueue(const char *name)
{
    int fds[2] = {-1,-1};

    pipe(fds);

    this->fd_read = fds[0];
    this->fd_write = fds[1];
    MsgQueName[0] = 0;
    strcat(MsgQueName, name);
    LOGD("%s create",name);
}
MessageQueue::~MessageQueue()           /* ddl@rock-chips.com */
{
    LOGD("%s destory",this->MsgQueName);
    close(this->fd_read);
    close(this->fd_write);

    this->fd_read = -1;
    this->fd_write = -1;
}

int MessageQueue::get(Message* msg)
{
    char* p = (char*) msg;
    unsigned int read_bytes = 0;

    while( read_bytes  < sizeof(msg) )
    {
        int err = read(this->fd_read, p, sizeof(*msg) - read_bytes);

        if( err < 0 ) {
            LOGE("%s.get error: %s", this->MsgQueName,strerror(errno));
            return -1;
        }
        else
            read_bytes += err;
    }

    MLOGD("%s.get(%s,%p,%p,%p,%p)", this->MsgQueName, MessageCmdConvert(this->MsgQueName,msg->command), msg->arg1,msg->arg2,msg->arg3,msg->arg4);

    return 0;
}

int MessageQueue::get(Message* msg, int timeout)
{
    char* p = (char*) msg;
    unsigned int read_bytes = 0;
    int err = 0;
    struct pollfd pfd;

    pfd.fd = this->fd_read;
    pfd.events = POLLIN;

    while( read_bytes  < sizeof(msg) )
    {
        pfd.revents = 0;
        err = poll(&pfd,1,timeout);

        if (err == 0) {
            LOGE("%s.get_timeout error: %s", this->MsgQueName,strerror(errno));
            return -1;
        }


        if (pfd.revents & POLLIN) {
            err = read(this->fd_read, p, sizeof(*msg) - read_bytes);

            if( err < 0 ) {
                LOGE("%s.get_timeout error: %s", this->MsgQueName,strerror(errno));
                return -1;
            } else {
                read_bytes += err;
            }
        }
    }
        MLOGD("%s.get_timeout(%s,%p,%p,%p,%p)",this->MsgQueName,  MessageCmdConvert(this->MsgQueName,msg->command), msg->arg1,msg->arg2,msg->arg3,msg->arg4);

    return 0;
}

int MessageQueue::put(Message* msg)
{
    char* p = (char*) msg;
    unsigned int bytes = 0;

    MLOGD("%s.put(%s,%p,%p,%p,%p)",this->MsgQueName, MessageCmdConvert(this->MsgQueName,msg->command), msg->arg1,msg->arg2,msg->arg3,msg->arg4);

    while( bytes  < sizeof(msg) )
    {
        int err = write(this->fd_write, p, sizeof(*msg) - bytes);

        if( err < 0 ) {
            LOGE("write() error: %s", strerror(errno));
            return -1;
        }
        else
            bytes += err;
    }

    return 0;
}


bool MessageQueue::isEmpty()
{
    struct pollfd pfd;

    pfd.fd = this->fd_read;
    pfd.events = POLLIN;
    pfd.revents = 0;

    if( 1 != poll(&pfd,1,0) ){
        return 1;
    }

    return (pfd.revents & POLLIN) == 0;
}

