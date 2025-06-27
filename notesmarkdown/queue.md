## C语言的链表
队列的预备知识
见[](/demo&test/code_draft/linklist.c)
你已经大致了解了
## C语言的队列
队列（Queue）是一种先进先出（FIFO）的线性数据结构，支持两种核心操作：
    入队（Enqueue）：在队列尾部添加元素
    出队（Dequeue）：从队列头部移除元素

|实现方式	|优点	|缺点	|适用场景|
|-----|-----|-----|----|
|数组队列	|内存连续、访问速度快	|固定大小、空间浪费	|已知最大容量的场景|
|链表队列|	动态扩容、内存利用率高	|需要指针、访问稍慢|	频繁增删的动态数据场景|

```c
/*节点就是队列的一个元素,我们规定这里的节点是int数字*/
// 队列节点结构
struct Node {
    int data;       //一个名为数据的int
    struct Node* next;//指向下一个节点,指针的类型是Node
};

// 队列容器结构
struct Queue {
    struct Node* front;//指向队列头部的指针
    struct Node* rear;//指向队列尾部的指针
    int size; //记录当前元素的数量
};

// 创建空队列
struct Queue* createQueue() {    //一个返回值是Queue结构体指针的函数
    struct Queue* q = malloc(sizeof(struct Queue)); //为队列分配内存，正式创建
    q->front = q->rear = NULL;   //初始化所有数值
    q->size = 0;    //初始化所有数值
    return q;       //返回一个队列的指针，用于访问
    //这时候队列只有一个节点，还没有正式存数据
}

// 入队操作
void enqueue(struct Queue* q, int item) {   //入队的函数,形参是新建队列的指针,和你要入队的数据
    struct Node* newNode = malloc(sizeof(struct Node)); // 为新的节点申请内存空间
    newNode->data = item; //设置新节点的数据
    newNode->next = NULL; //这时候新的节点是最后一个节点，后面没有了，改成空指针
    
    if (q->rear == NULL)   //如果队列是空的（这里rear和front类似的）
    {    
        q->front = q->rear = newNode; //新节点是唯一的节点，也就是说我们规定队列不允许有空数据，是上面的链表程序的优化
    } 
    else 
    {
        q->rear->next = newNode;   //将当前的尾部节点的next指向新节点，将新的节点接入链表
        q->rear = newNode;  //这时候新的节点成为了最后一个充当新的rear
    }
    q->size++;  //队列元素加一
}
```

![](/Image/链表解析.png)

```c
// 出队操作
int dequeue(struct Queue* q) {   //形参是队列结构体指针
    if (q->front == NULL) return -1;//如果数列是空,返回错误，说明你从来没有入队
    
    struct Node* temp = q->front;  //备份头部节点，front是要反复使用的
    int item = temp->data;   //备份数据
    
    q->front = q->front->next;  //将front移动到下一个节点，解除链表的联系
    if (q->front == NULL)  //如果队列空，也就是你把最后一个数据出队了，那么链表退化成了初始状态，front和rear都是NULL
        q->rear = NULL;  //要手动设置，否则rear和原来的front是一样的
    
    free(temp);    //释放内存，解除联系后彻底删除
    q->size--;   //队列元素-1
    return item;  //返回出队的元素值
}
```
自己运行一下试试吧