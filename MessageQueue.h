#ifndef __MESSAGEQUEUE_H__
#define __MESSAGEQUEUE_H__

struct Message
{
    unsigned int command;
    void*        arg1;
    void*        arg2;
    void*        arg3;
    void*        arg4;
};

class MessageQueue
{
public:
    MessageQueue();
    MessageQueue(const char *name);
	~MessageQueue();			/* ddl@rock-chips.com */
    int get(Message*);
	int get(Message*, int);		/* ddl@rock-chips.com : timeout interface */
    int put(Message*);
    bool isEmpty();
    int dump();
private:
    char MsgQueName[30];
    int fd_read;
    int fd_write;
};

#endif

