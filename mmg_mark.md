

## Linux内核

#### 进程与线程

1. 进程与线程的区别:

   > * 进程是资源分配的基本单位，拥有独立的进程地址空间，一个进程包含多个线程
   > * 线程是资源执行调度的基本单位；没有独立的进程地址空间，地址空间共享；全局变量共享
   
2. 进程状态

   > * 运行：占用cpu
   > * 就绪：等待cpu分配时间片
   > * 挂起：等待除cpu外其他资源主动放弃cpu
   > * 停止:

<img src="/home/gmm/下载/c++_programming/Linux进程4G图.png" alt="Linux进程4G图" style="zoom: 67%;" />


3. fork函数

   > * fork执行之后，子进程并非都要将父进程的用户区拷贝，而是遵循读时共享，写时复制
   > * 刚fork之后，**父子进程相同：**data段，text段，堆，栈，环境变量，宿主目录位置，进程工作目录位置，信号处理方式
   > * **父子进程不同：**进程id、返回值、各自的父进程、进程创建时间、闹钟、未决信号集
   > * **父子进程共享：** 文件描述符，mmap映射区

4. 进程分类

   > * 孤儿进程：父进程先于子进程结束，子进程变为孤儿进程，子进程的父进程变成init进程
   > * 僵尸进程：用 fork 创建子进程，如果子进程退出，而父进程并没有调用 wait 或 waitpid 获取子进程的状态信息，那么子进程的进程描述符仍然保存在系统中
   > * 守护进程

5. 避免僵尸进程：

   > * 杀死父进程
   > * 在父进程中使用wait或者waitpid回收子进程



#### 进程之间的通信方式

本质是内核空间中的一块缓冲区

1. **管道**

   > * 本质是伪文件(实为内核缓冲区)
   > * 只能用于有血缘关系的进程之间通信
   > * 由两个文件描述符引用，一个表示读端，一个表示写端
   > * 规定数据从管道的写端流入，读端流出，双向半双工

1. **信号**
2. **共享内存**
3. **本地套接字**



#### 虚拟地址

```c
//1) 假如程序中访问数组元素，这个数组a生成一个虚拟地址0x00 00 12 34
//2) MMU从虚拟地址中提取页号和偏移量
//3) 假设页面大小为4kb，页号为0x00 00 ，偏移量是0x12 34，寄存器通常大小为2^64
//4) MMU使用页号去页表中查找，找到对应的物理页面帧号 0x0A
//5) 最终生成的物理地址为0x0A 12 04
//6) 内存控制器，从而访问实际的物理内存位置。
//不同进程的内核区映射到同一块物理内存
```



#### 用户区与内核区转换

1. **文件读取**

   ```c
   //当用户程序调用read()时，控制权会从用户区转移到内核区，内核负责从磁盘读取数据，并将数据复制到用户程序的内存中。
   #include <stdio.h>
   #include <fcntl.h>
   #include <unistd.h>
   
   int main() {
       char buffer[100];
       int fd = open("file.txt", O_RDONLY);
       ssize_t bytesRead = read(fd, buffer, sizeof(buffer));
       close(fd);
   
       printf("Read %zd bytes: %s\n", bytesRead, buffer);
       return 0;
   }
   ```

2. **除零错误**

   ```c
   #include <stdio.h>
   //如果用户程序试图执行除以零的操作，会触发一个除零错误异常。这时，控制权会从用户区转移到内核区，内核会处理这个异常，通常会导致程序终止或其他适当的处理。
   int main() {
       int a = 10;
       int b = 0;
       int result = a / b;  // 触发除零错误异常
       printf("Result: %d\n", result);  // 不会执行到这里
       return 0;本地套接字
   }
   ```

3. **信号处理示例：接收Ctrl+C信号**

   用户程序可以注册信号处理函数，用于处理特定的信号。例如，当用户按下Ctrl+C组合键时，操作系统会发送一个SIGINT信号，用户程序可以注册一个信号处理函数来捕获并处理这个信号。

   ```c
   #include <stdio.h>
   #include <signal.h>
   
   void handleSignal(int signal) {
       printf("Received signal %d (Ctrl+C)\n", signal);
   }
   
   int main() {
       signal(SIGINT, handleSignal);  // 注册信号处理函数
       printf("Press Ctrl+C to send SIGINT...\n");
       while (1) {
           // 程序持续运行
       }
       return 0;
   }
   ```

   

#### 死锁

1. **定义**：当多个线程访问共享资源，最终形成互相等待都被阻塞的现象

2. **情况**：

   > * 加锁忘记解锁
   > * 重复加锁
   > * 线程A已经加锁持有资源X，线程B已经加锁持有资源Y；此时线程A尝试去访问资源Y，线程B尝试去访问资源X

 3. **避免死锁**：

       > * 避免使用多个锁，多检查
       > * 使用trylock
       > * 资源合理分配，控制线程顺序线性访问共享资源；加锁之前释放当前线程所拥有的互斥锁
       > * 引入一些专门用于死锁检测的模块



#### 条件变量

1. **语法：**

   ```c++
   pthread_cond_t cond;	//创建
   pthread_cond_init(&mutex, NULL);	//初始化
   pthread_cond_wait(&cond, &mutex);	//阻塞消费者
   pthread_cond_broadcast(&cond);		//唤醒所有线程
   pthread_cond_signal(&cond);			//唤醒单个线程
   ```

2. **注意事项：**

   > * pthread_cond_wait启动之后，该线程会释放所拥有的锁
   > * 生产者线程抢夺互斥锁执行任务，然后再进行pthread_cond_broadcast唤醒消费者
   > * 消费者线程抢夺互斥锁资源，加锁成功后继续向下执行，没有加锁成功则继续阻塞



#### pcb进程控制块

> * 进程id
> * 文件描述符
> * 进程状态
> * 进程工作目录位置
> * umask掩码
> * 信号相关变量
> * 用户id和组id



#### 系统调用

> * 操作系统实现并提供给外部应用程序的访问底层系统功能的编程接口
> * 除了异常和陷入之外，是内核唯一的合法入口

<img src="https://img-blog.csdnimg.cn/dc67ded4de4142259612a3ebca7eeeba.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBATmVpbF96aw==,size_20,color_FFFFFF,t_70,g_se,x_16" alt="img" style="zoom: 67%;" />



#### 存储映射

存储映射I/O (Memory-mapped I/O) 使一个磁盘文件与存储空间中的一个缓冲区相映射



#### 线程同步方式

1. 互斥锁：
2. 条件变量：
3. 信号量：控制多个线程对共享资源的访问
4. 自选锁：忙等待同步机制，线程不会休眠，而是不断检查锁是否可用



#### 上下文切换

> * 进程线程分时复用CPU时间片
> * 在切换之前会将上一个任务的状态保存
> * 下次切换回任务时候，加载这个状态运行
> * 任务从保存到再次加载这个过程就是一次上下文切换



#### 线程数目控制

> * 文件IO操作：文件IO对CPU使用率不高，因此可以分时复用CPU时间片，线程个数=2*CPU核心数
> * 处理复杂的算法（主要是CPU进行运算）



#### 主线程sleep(3)

> * 主线程挂起
> * 放弃cpu资源



#### 线程退出

> * void pthread_exit(void *retval);
> * 主线程退出不影响子线程运行



#### 线程回收

> * int pthread_join(pthread_t thread, void **retval);
> * 一直等待子线程执行完毕并退出
> * pthread_join()只要子线程不退出主线程就会一直被阻塞，主线程的任务不能执行
> * 主线程回收子线程资源
> * 每个join只能阻塞一个子线程



#### 子线程传参

```c
struct Test{
    int num;
    int age;
};
//方法一
//struct Test t;
void *callback(void* arg){
    for(int i = 0; i < 5; ++i){
        printf("子线程：i == %d\n", i);
    }
    printf("子线程：%ld\n", pthread_self());

    struct Test* t = (struct Test*)arg;
    t->num = 100;
    t->age = 16;
    pthread_exit(t);
    return NULL;
}

int main(){
    struct Test t;
    pthread_t tid;
    pthread_create(&tid, NULL, callback, &t);
    printf("主线程：%ld\n", pthread_self());
	
    //方法一
    /*
    void *ptr;
    pthread_join(tid, &ptr);
    struct Test* pt = (struct Test*)ptr;
    printf("num:%d, age = %d\n", pt->num, pt->age);
    */
    
    //方法二
    void *ptr;
    pthread_join(tid, &ptr);
    printf("num:%d, age = %d\n", t.num, t.age);
    return 0;
}
```



#### 线程分离

> * int pthread_detach(pthread_t thread);
> * 子线程和主线程分离
> * 子线程退出，其占用的内核资源被系统其他接管并回收了
> * 线程分离之后在主线程中使用pthread_join回收不到子线程资源了



#### 线程取消

> * int pthread_cancel(pthread_t thread);
> * 线程取消是在某些特定情况下在一个线程中杀死另一个线程
> * 分为两步：1）线程A中调用线程取消函数pthread_cancel，指定杀死线程B；  2）在线程B中进行一次系统调用



#### 线程同步

> * 原因：数据没来得及从物理内存写入寄存器



#### 互斥锁

> * pthread_mutex_t mutex
> * int pthread_mutex_init(pthread_mutex_t *restrict mutex, const pthread_mutexattr_t *restrict attr);
> * int pthread_mutex_destroy(pthread_mutex_t  *mutex);
> * int pthread_mutex_lock(pthread_mutex *mutex);
> * int pthread_mutex_trylock(pthread_mutex_t *mutex);

关于trylock，如果没有加锁，则加锁成功，如果已锁，调用这个函数加锁的线程，不会被阻塞，加锁失败直接返回错误信号



#### 读写锁

> * pthread_rwlock_t rwlock;
> * int pthread_rwlock_init(pthread_rwlock *restrict rwlock, const pthread_rwlockattr_t *restrict attr);
> * int pthread_rwlock_destroy(pthread_rwlock_t *rwlock);



#### 条件变量

处理生产者消费者模型，阻塞消费者线程，打开互斥锁，生产者抢到互斥锁，才能去生产



#### 信号量

用在多线程多任务同步的，一个线程完成了一个动作就通过信号量告诉别的线程，别的线程再进行某些动作，信号量不一定是锁定某个线程，而是流程上的概念。

> * int sem_init(sem_t *sem, int pshared, unsigned int value);
> * int sem_destroy(sem_t *sem);
> * int sem_wait(sem_t *sem);



#### 进程切换

> * 保存上下文：进程状态，包含寄存器值，程序计数器，内存映射，文件描述符，PCB进程控制块
> * 选择新进程：操作系统根据调度算法从就绪队列中选择下一个要运行的进程
> * 加载新进程的上下文
> * 切换内存映射表：不同进程有不同的地址空间



#### PCB进程控制块

> * 进程id
> * 进程状态
> * cpu寄存器



#### 管道通信

> * 本质是内核的缓冲区
> * 由两个文件描述符引用，一个表示读端，一个表示写端
> * 规定数据从管道的写端流入管道，从读端流出，环形队列
> * 半双工通信，不可重复读写
> * 只能在有血缘关系的进程间使用管道

**读管道：**

 	1. 管道有数据：read返回实际读到的字节数 
	2. 管道无数据：1）无写端，read返回0；2）有写端，read阻塞等待

**写管道：**

1. 无读端，异常终止。(SIGPIPE导致的)
2. 有读端：1）管道已满，阻塞等待；2）管道未满，返回写出的字节个数

## C++和C语言

#### C++内存分区

> * 堆区：程序员手动管理
> * 栈：局部变量、函数参数、返回地址；系统分配
> * 全局区：全局变量，静态变量，程序结束之后释放
> * 常量存储区：存储常量，如字符串常量
> * 代码区：存储程序的执行代码。内存只读，存放机器码



#### 函数指针

1. **用法：**

   ```c
   // 声明一个函数原型
   int add(int a, int b) {
       return a + b;
   }
   
   int subtract(int a, int b) {
       return a - b;
   }
   
   int main() {
       // 声明函数指针类型，该指针可以指向带有两个int参数和int返回值的函数
       int (*f)(int, int);
   
       // 将函数指针指向add函数
       f = add;
       printf("Result: %d\n", f(5, 3)); // 输出：8
   
       // 将函数指针指向subtract函数
       f = subtract;
       printf("Result: %d\n", f(10, 4)); // 输出：6
   
       return 0;
   }
   ```

   2. **特点：**

      > * 回调函数：可将一个函数作为参数传递给另一个函数，从而实现回调机制
      > * 函数动态调用：可以在运行的时候根据不同条件动态调用不同函数，而非编译时确定调用哪个函数

      

#### Auto自动类型推导

**1.注意事项：**

> * 使用必须初始化
> * 不能用于函数形参中的定义
> * 不能用于定义数组
> * 不能用于类模板函数模板



#### 左值与右值

1. 定义：

   > * 左值是指可以被标识，可以被寻地址的值，既可以在等号左边也可以在等号右边
   > * 右值是指不能被标识，不能被寻址的值，只能在等号右边

2. 用法：

   ```c++
   int num = 9;	//左值
   int &a = num;	//左值引用
   
   int &&b = 8;	//右值与右值引用
   
   const int& c = num;	//常量左值引用
   const int&& d = 6;	//常量右值引用
   ```

3. 特点：

   > * 使得将亡值重获新生，减少内存的复制拷贝开销

4. 移动构造函数：

   ```c++
   Person(Person&& person1):m_num(person1.age){
       person1.age = nullptr;
       cout << "move construct...." << endl;
   }
   //移动构造函数将移动源对象的资源所有权转移到正在构造的新对象中;
   //同时保留移动源对象的状态，使其进入有效但未定义的状态。这样做避免了资源复制操作，提高了性能。
   
   Person p1;  // 创建一个对象
   Person p2(std::move(p1));  // move将p1变为将亡值，资源转移，没有内存拷贝
   ```

   

   #### 智能指针

   1. **shared_ptr**

      ```c++
      //初始化
      shared_ptr<int> ptr1(new int(2));
      shared_ptr<int> ptr2 = move(ptr1);
      shared_ptr<int> ptr3 = ptr2;
      shared_ptr<int> ptr4 = make_shared<int>(2);
      shared_ptr<Person> ptr5 = make_shared<Person>("Tom");
      
      //内存释放
      ptr6.reset();
      ptr5.reset(new int(99));	//error
      ptr5.reset(new Person("Jack"));	//析构之后并重新构造
      cout <<ptr5.use_count() << endl;	//输出1
      
      //1.获取原始指针操作
      Person* p = ptr5.get();
      p->setAge(16);
      p->print();
      
      //2.直接操作
      ptr5->setAge(16);
      ptr5->print();
      
      //如果是数组类型，需要手动指定删除器函数，释放内存
      //1.通过lambda表达式
      shared_ptr<Test>p1(new Person[5], []Test*t){
          delete[]t;
      }
      //2.通过默认删除器
      shared_ptr<Person>p2(new Person[5], default_delete<Person[]>());
      ```

   2. **unique_ptr**

      ```c++
      //初始化
      unique_ptr<int> ptr1(new int(2));
      unique_ptr<int> ptr2 = move(ptr1);
      ptr2.reset(new int(8));
      
      //获取原始指针
      unique_ptr<Person> ptr3(new Person(1));
      Person* pt = ptr3.get();
      pt->setAge(16);
      pt->print();
      
      //直接操作
      ptr3->setAge(16);
      ptr3->print();
      ```




#### 虚函数与多态

<img src="/home/gmm/.config/Typora/typora-user-images/image-20230902193801064.png" alt="image-20230902193801064" style="zoom:33%;" />

> * 父类子类都有一个虚函数表指针vptr指向各自的虚函数表
> * 虚函数表vptl里面存放的是指针指向不同的虚函数
> * 当子类重写父类虚函数时，虚函数表里面的指针就被替换为指向子类的虚函数



#### 移动语义与完美转发

1. 移动语义

   > * 定义：在对象的所有权转移时，避免不必要的数据复制操作，将资源的所有权从一个对象转移到另一个对象，提高程序效率
   > * 特点：普通的复制操作会复制所有成员函数；移动语义通过使用右值引用和移动构造函数实现资源的高校转移，避免了资源的不必要复制

2. 完美转发

   > * 定义：在函数模板中将参数按照原始类型转发给其他函数，无论参数时左值还是右值，都能保持其原有的左值或右值特性。
   > * 特点：完美转发使得我们能够编写通用的函数模板，能够正确地传递参数的值特性，避免了代码中的冗余和重复。

3. 语法

   ```c++
   //1.移动语义
   class MyObject {
   public:
       MyObject() { std::cout << "Default constructor" << std::endl; }
       MyObject(const MyObject& other) { std::cout << "Copy constructor" << std::endl; }
       MyObject(MyObject&& other) { std::cout << "Move constructor" << std::endl; }
   };
   
   int main() {
       std::vector<MyObject> vec;
   
       MyObject obj1;
       vec.push_back(obj1);  // 调用复制构造函数
   
       MyObject obj2;
       vec.push_back(std::move(obj2));  // 调用移动构造函数
   
       return 0;
   }
   
   ---------------------------------------------------------------
       
   //2.完美转发
   void process(int& value) {
       std::cout << "Lvalue reference: " << value << std::endl;
   }
   
   void process(int&& value) {
       std::cout << "Rvalue reference: " << value << std::endl;
   }
   
   template <typename T>
   void forward(T&& value) {
       process(std::forward<T>(value));
   }
   
   int main() {
       int x = 42;
   
       forward(x);       // 传递左值，调用 Lvalue reference 版本的 process
       forward(123);     // 传递右值，调用 Rvalue reference 版本的 process
   
       return 0;
   }
   ```



#### new和malloc区别

> * new是c++运算符，malloc是c语言函数
> * new不需要显式指定内存大小，malloc需要
> * new分配内存失败抛出std::bad_alloc，malloc返回NULL
> * new可以重载，malloc不能重载
> * new释放内存调用析构函数，malloc不调用
> * new释放数组内存delete[]，malloc用free



#### strlen和sizeof区别

> * sizeof是c++运算符，strlen为c库函数
> * sizeof常用于c++中变量所占内存适用于几乎任何的数据类型，strlen主要用于计算字符串的实际长度，字符串通常以NULL结尾
> * strlen("hello") = 5



#### strcpy特点

> * strcpy常用于将源字符串内容复制到目标字符串，包括null终止符号
> * 会一直复制直到遇到源字符串的null终止符
> * 不会检查目标字符串内存是否可以容下源字符串，如果目标内存不够，容易造成缓存溢出
> * 可以用strncpy或者strlcpy来确保不会导致缓冲区溢出，确保源字符串有null终止符号



#### char*和char[]区别

> * char*声明一个指向字符指针；
> * char[]声明一个字符数组，是一块连续的内存区域
> * char*所指向可以变，char[]一旦声明后，内存和指向不能更改



#### 指针数组与数组指针

1. 指针数组

```c
//指针数组，相当于10个int*，int*, int*, int*.......
int *p[10]; //or char*p[10]

//用法
char *names[] = {"Alice", "Bob", "Charlie", "David"};
for (int i = 0; i < 4; i++) {
    printf("Name: %s\n", names[i]);
}
```

2. 数组指针

   ```c
   //数组指针，相当于10个int,int.......
   int (*p)[10];//or char (*p)[10]
   
   //用法
   int matrix[2][3] = {{1, 2, 3}, {4, 5, 6}};
   int (*ptrToMatrix)[3] = matrix;
   for (int i = 0; i < 2; i++) {
       for (int j = 0; j < 3; j++) {
           printf("%d ", ptrToMatrix[i][j]);
       }
       printf("\n");
   }
   ```

3. 指针与数组

```c
//方法1
char array[] = "Hello, world!";
char *p = array;
while(*p != '\0'){
    cout << *p << endl; //输出Hello, world!
    cout << p << endl;	//输出Hello, world!,ello, world!, .....!
    p++;
}
return 0;

//方法2
int arr[] = {1, 2, 3, 4, 5};
int *ptr = &arr[0]; // 使用取地址运算符
```

4. 指针与数组结构体

```c
typedef struct{
  int a;
  char ptr;
}Point;

int main(){

  Point p1[2];
  p1[0].a = 10;
  p1[0].ptr = 'A';
  p1[1].a = 20;
  p1[1].ptr = 'B';

  Point *pointer = p1;	//or Point *pointer = &p1[0];
  int count = 2;
  while(count--){
    cout << "a: " << pointer->a << " ptr: " << pointer->ptr << endl;
    //or cout << (*pointer).a << (*pointer).ptr << endl;
    //error: *pointer.a
    pointer++;
  }

  return 0;
}

-------------------------------------------------------------------
void test01(){
	int array[] = {2, 2, 3, 11, 5};
	int *p = array;
	for(int i = 0; i < 5; ++i){
		cout << "Address of pointer ptr: " << &p << endl;
		// cout << "*p++: " << *p++ << endl;	//2, 2, 3, 11, 5
		//cout << " (*p)++: " << (*p)++ << endl;	//2, 3, 4, 5
		cout << " (*p)++: " << *(p++) << endl;		//2, 2, 3, 11, 5
	}
}
```



#### C语言中的struct结构体

1. 字节对齐

```c
//struct结构体在c语言中所占用内存
struct status{
    char name[20];		//24byte,   20byte padding 4byte;
    unsigned int *ptr;	//8byte
    char tel[15];		//15byte
    char sex;			//1byte
};

//1.找到最大的变量所占用的内存a，并且检查是否为内存基本单位的整数倍，如果不是进行补齐
//2.将其他变量按照a的整数倍进行排列

//例：64位操作系统中
struct test01{
    int a;  //4byte
    short b;    //2byte
    char c;     //2byte padding 1byte
    unsigned int *d;    //最大8byte
    char e;     //e和f公用8byte
    short f;
};
```

2. pragram pack

   > * 告诉编译器以指定的字节对齐方式来分配结构体内存
   > * 例如使用#pragma pack(1)指令将内存对齐方式设置为最小1字节对齐
   > * 某些硬件架构要求特定的对齐方式获得最佳性能，使用非标准的对齐方式可能会使得代码不可以移植

3. 字节拼接

```c
//Timestamp-us
/* utc时间 单位 微秒 */
//小端排列
long timestamp1 = tv.tv_sec*1000*1000 + tv.tv_usec;
packet[18] = (timestamp1) & 255;	//低
packet[19] = (timestamp1>>8) & 255;
packet[20] = (timestamp1>>16) & 255;
packet[21] = (timestamp1>>24) & 255; //高
```



#### Union结构体

> * 不同数据成员共享同一块内存
> * 修改一个变量也会影响另一个变量
> * struct结构体里面的变量是相对独立的
> * 联合体的大小至少是最大成员大小，当最大成员大小不是最大对齐数整数倍的时候，就要对齐到最大对齐数的整数倍



#### Volatile关键字

预处理阶段告诉编译器不要对变量优化

> * 多线程应用中，多个线程可能同时访问和修改同一变量，修饰变量可以确保不会被编译器优化
>
> * 硬件编译器
> * 信号处理函数



#### Override关键字

> * 显示指示派生类中的成员函数覆盖基类的虚函数，目的是提高代码的可读性和可维护性
> * 同时在编译时捕捉一些常见的错误，如拼写错误、参数不匹配等。



#### extern 关键字

> * extern "C"告诉编译器以C方式对待指定的代码块或者函数接口，遵循C语言的命名和链接
> * 在另一个文件中，引用该文件变量，声明全局变量或函数
> * 避免重复编译



#### void*

> * 无类型的指针,可以指向任意类型的数据；
>
> * 但是不能直接用于访问所指向的数据，使用之前需要将其转换为特定的类型



#### const关键字

> * c++中用来定义常量
> * 可用于限制变量的修改，将其声明为只读属性
> * 修饰指针

```c++
//常量指针
int x = 5;
const int *p = &x;	//不能通过p修改x的值

//指针常量
int x = 5;
int *const p = &x;	//不能修改指针的指向

```



#### 引用

> * 变量的别名
> * 必须初始化变量才能绑定，且不能根该绑定对象
> * 修改引用会修改源变量值
> * 可以作为函数参数传递避免对象的拷贝，实际传地址，提高性能



#### 指针

> * 是变量，变量里面存储的是内存地址
> * 必须初始化



#### 指针和引用的区别

> * 初始化语法不同
> * 指针存储的是变量内存地址，通过*，获取地址中的值，引用是变量别名，没有解引用
> * 可以有空指针，但不能有空引用
> * 普通指针可以修改指针指向，而引用不能修改绑定对象
> * 函数传参的时候，指针传递的是指针变量的副本，引用传递的是地址



#### 重载与重写

> * 重载是函数名相同函数返回类型相同，形参列表不同，是静态多态，发生在编译时刻
> * 重写是实现动态多态时，函数名相同，形参列表完全相同，发生在运行时
> * 重载返回类型必须相同，栈空间会增加，否则会发生歧义



#### 函数调用

假如c++中A函数调用B函数，栈上如何实现

> * 保存调用者上下文：在函数A调用函数B之前，函数A的状态（局部变量、返回地址）保存到栈上
> * 参数传递：先将局部参数进行拷贝，然后将参数入栈
> * 调用函数B：函数A的地址压入栈中，控制权转移给调用者B
> * 函数B的执行：函数B执行，同时将函数内部的局部参数入栈
> * 返回值和恢复：函数B执行完毕，将返回值存放在特定的地址区域，然后将B之前的局部变量出栈，恢复A的上下文
> * A的地址出栈，将控制权还给调用者





## 计算机网络

#### UDP通信

1. **UDP丢包**

   > * 调整发送的速率和缓冲区
   > * 实现自定义的重传机制，当发送方发送数据包后，如果没有收到接收方的确认，发送方就可以选择重传数据包 
   > * 给每个发送的数据包分配一个唯一的序列号，接收方可以根据序列号判断是否有丢包情况发生，从而请求发送方重传丢失的数据包。 
   > * 实施日志记录和网络监控，以及实时监测网络性能，有助于及时发现丢包问题并采取相应的措施。 

2. **大小端**

   ```c
   //假设0X789a从左到右为高->低
   //则大端排列为
   A[0] = 0x78;
   A[1] = 0x9a;
   //小端排列为
   A[0] = 0x9a;
   A[1] = 0x78;
   
   //将一个短整型从主机字节序->网络字节序
   uint16_t htons(uint16_t hostshort);	
   //将一个整形从主机字节序->网络字节序
   uint16_t htonl(uint32_t hostlong);
   
   //将一个短整型从网络字节序->主机字节序
   uint16_t ntons(uint16_t hostshort);	
   //将一个整形从网络字节序->主机字节序
   uint16_t ntonl(uint32_t hostlong);
   
   //将点分十进制IP转换为大端整形
   in_addr inet_addr(const char *cp);
   //大端整形->点分十进制IP
   char inet_ntoa(struct in_addr in);
   ```



#### OSI七层模型

1. **物理层**：建立、维护及断开物理连接

2. **数据链路层**：建立逻辑连接、硬件地址寻址、差错校验等功能

3. **网络层**：进行逻辑地址寻址，实现到达不同网络的路径选择

   > * IP协议：互联网协议，定义网络层的地址
   > * ICMP：网络控制消息协议，探测网络连接情况
   > * ARP：地址解析协议，将IP地址协议解析为MAC地址
   > * IGMP：网络组消息协议

4. **传输层**：定义传输数据的协议端口号，以及流控和差错校验

   > * TCP：
   > * UDP：

5. **会话层**：建立、管理终止会话

6. **表示层**：数据的表示、安全、压缩、加密等

7. **应用层**：网络服务于最终用户的一个接口

   > * HTTP：超文本传输协议，FTP，TFTP，SMTP，SNMP
   > * DNS：域名服务



#### VLAN作用

1. **定义**：虚拟局域网，是将一个物理的LAN在逻辑上划分为多个广播域的通信技术，VLAN内的主机间可直接通信，而VLAN间不能直接通信，而VLAN间不能通信，从而将广播报文限制在一个VLAN内
2. 作用：限制广播域，广播被限制在一个VLAN内，节省了带宽，提高了网络处理能力

<img src="https://img-blog.csdnimg.cn/35c0c159846b4922a6c3f16ab5af48ea.png" alt="img" style="zoom:50%;" />



#### 查看socket编程的ip和端口状态

> * netstat
> * ss -tuln



#### Github配置

```shell
1.git config --global user.name "Mmg-qL"
2.git config --global user.email "gmm782470390@163.com"
3.cd ~/.ssh
4.ssh-keygen -t rsa -C "gmm782470390@163.com"
5.将Enter file in which xxx: id_rsa；其他一直enter
6.cat id_rsa.pub
7.打开Github中的SSH and GPG keys
8.添加密钥输入，并将生成的ssh-key粘贴
```



## 调试相关

#### Autoware

1. **lidar_point_pillars启动**

   > * cd /usr/file/autoware.ai/install
   > * source setup.bash
   > * 修改rosbag包以及话题的名称
   > * 打开之后修改rviz的frame

2. **autoware_msgs::DetectedObjectArray**

   > * header:消息时间戳、帧ID等信息，时间同步和数据标识
   > * objects:包含多个autoware_msgs::DetctedObject对象数组
   >   		* header
   >     		* id
   >        			* label
   >        			* pose
   >                  		* velocity

3. **autoware_msgs::DetectedObject**

   用于表示检测到物体信息的ROS消息类型

   > * header:消息的时间戳、帧ID
   > * id：物体的唯一标识符
   > * label：物体标签
   > * socre：检测置信度
   > * pose：位置姿态，通常以三维坐标和姿态形式表示
   > * dimensions：尺寸
   > * velocity：线速度和角速度
   > * accleration：线加速度和角加速度
   > * cloud：点云信息
   > * convex_hull：通常包括凸包的顶点坐标，用于描述物体的轮廓。

4. **tf::StampedTransform**

用于监听并查询这个类型的变换消息，包含两个坐标系之间的变换关系，包括旋转和平移。此外还包含时间戳信息，用于表示变换时间

```c++
tf::TransformListener listener;
tf::StampedTransform transform;
try {
    listener.lookupTransform("odom", "base_link", ros::Time(0), transform);
    // 查询成功，可以使用 transform 获取变换信息
} catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
}
//在上述示例中，我们尝试查询从 "odom" 到 "base_link" 坐标系的变换信息，并将结果存储在 transform 中。如果查询失败，将会抛出 tf::TransformException 异常。
```



#### Autoware相机与雷达检测程序启动

```shell
#rosbag播放
rosbag play -l kitti_2011_10_03_drive_0047_synced.bag /kitti/camera_color_left/image_raw:=/image_raw /kitti/velo/pointcloud:=/points_raw
    
 rosbag play -l /usr/file/autoware.ai/src/autoware/rosbag/kitti_2011_10_03_drive_0047_synced.bag /kitti/velo/pointcloud:=/points_raw
    
 rosbag play -l CADCD_2019_02_27_seq_0051 /cadcd/velo/pointcloud:=/points_raw /cadcd/2019_02_27/0051/camera_00/image_raw:=/image_raw
 
 rosbag play -r 0.5 -l 2023-11-25-12-29-54.bag /lslidar_point_cloud:=/points_raw /zed/zed_node/rgb/image_rect_color:=/image_raw
 
 #编译
 AUTOWARE_COMPILE_WITH_CUDA=1 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
 
 #roslaunch包
roslaunch lidar_point_pillars lidar_point_pillars.launch pfe_onnx_file:=/usr/file/autoware.ai/src/autoware/core_perception/lidar_point_pillars/test/data/kitti_pretrained_point_pillars/pfe.onnx rpn_onnx_file:=/usr/file/autoware.ai/src/autoware/core_perception/lidar_point_pillars/test/data/kitti_pretrained_point_pillars/rpn.onnx

```



#### Autoware中的权重文件找不到报错

* 需要将配置文件复制一份到install/xxxxbag/share目录下面



#### ZED相机驱动安装

```c
1. 官网下载相应的.run文件
2.  chmod +x abc.run
3. ./abc.run
4. python 
5. import pyzed检查能否成功导入
6. cd /usr/local/zed/tools
7. ./ZED_Explorer 
8../ZED_Depth_Viewer
9. mkdir ~/catkin_ws/src
10. cd catkin_ws/src
11. catkin_init_workspace
12. cd ..
13. catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
14. git clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git
15. cd ..
16. rosdep install --from-paths src --ignore-src -r -y
17. catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=Release
18. source devel/setup.bash
19. roslaunch zed_wrapper zed.launch
```



#### Autoware编译报错

* 注释部分代码，逐个模块进行调试



#### autoware中的跟踪代码检测框yaw角度问题

* 熟悉打印log
* 熟悉代码
* 跟踪的yaw角度较差，修改输出的更新让其直接等于检测的yaw角度
* object.angle输出为0，但是发现object.pose.orientation不为0
* 代码bug，之前添加的部分忘记删除



#### autoware中的障碍物坐标转换到路径规划错误问题

* 调试heading角度观察变化
* 打印正确的角度
* 发现矩阵写反



#### autoware中的相机雷达融合问题

* 打印log
* 检查相机内参矩阵输入错误



#### autoware中加入GPS和IMU的位置没有跟踪的问题

* 打印log定位问题到det > threshold这个条件将target给过滤掉
* 将条件改为isnan



#### 调试编译bug

1. 数组下标越界

   ```c++
   //1.加加与减减出错导致下标超出范围
   for(int i = bagSize; i >= 0; i++)
       
   //2.下标出现负数
   for(int i = 0; i < n; i++){
       for(int j = bag; j >= 0; j--)
           dp[j] = dp[j - num[i]];
   }
   //3.动态规划类题目，数组没有初始化
   ```



#### 镭神32线雷达启动

> * mkdir -p ~/lisaer_lidar/src
> * cd /lisaer_lidar
> * catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
> * source devel/setup.bash
> * roslaunch lslidar_driver lslidar_c32.launch
> * 修改fixed_frame 为laser_link



#### Linux环境下的pthread编程

1. 编译

   > * g++ test.cpp -o t_exe -lpthread



#### 调试定位问题

> * 打印log信息定位代码在哪一行出错
> * 监控相应的传感器信息



#### Nvidia驱动问题

**问题描述：**

输入Nvidia-smi显示报错NVIDIA-SMI has failed because it couldn't communicate with the NVIDIA driver. Make sure that the latest NVIDIA driver is installed and running.这种情况多数是驱动不匹配

**解决方法:**

```shell
#方法一
sudo apt-get remove --purge nvidia*  
sudo apt autoremove
sudo gedit /etc/modprobe.d/blacklist.conf 或者(blacklist-nouveau.conf)
在文件末尾添加：
blacklist nouveau
options nouveau modeset=0
更换不同的源文件，在附加更新中选择不同的驱动版本
reboot
#方法二
检查主板的security boot是否设置为disable，发现解决问题
#方法三
进入英伟达官网下载自己手动安装驱动

```

[【超详细】【ubunbu 22.04】 手把手教你安装nvidia驱动，有手就行，隔壁家的老太太都能安装_ubuntu安装nvidia显卡驱动-CSDN博客](https://blog.csdn.net/huiyoooo/article/details/128015155?spm=1001.2014.3001.5506) 

[【Ubuntu 20.04安装和深度学习环境搭建 4090显卡】_ubuntu20.04安装40系显卡驱动-CSDN博客](https://blog.csdn.net/qq_43775794/article/details/131770933?spm=1001.2014.3001.5506) 

[错误 NVIDIA-SMI has failed because it couldn’t communicate with the NVIDIA driver. 解决方案-腾讯云开发者社区-腾讯云 (tencent.com)](https://cloud.tencent.com/developer/article/2067457) 



#### cuda和cudnn安装

* [Ubuntu20.04下CUDA、cuDNN的详细安装与配置过程（图文）_ubuntu cudnn安装-CSDN博客](https://blog.csdn.net/weixin_37926734/article/details/123033286?ops_request_misc=&request_id=&biz_id=102&utm_term=ubuntu%20%E5%AE%89%E8%A3%85cuda%E5%92%8Ccudnn&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-3-123033286.142^v99^pc_search_result_base9&spm=1018.2226.3001.4187) 
* [Linux/Ubuntu系统如何安装cudnn？（适用于所有40系显卡 4090 4080 4070）_4090如何使用cudnn提速-CSDN博客](https://blog.csdn.net/weixin_45941288/article/details/129787559) 



#### TensorRT安装

* [记ubuntu18.04安装使用TensorRT_ubantu安装tensorrt18.4的保姆级教程-CSDN博客](https://blog.csdn.net/laizi_laizi/article/details/121567609) 
* 安装在配置的虚拟环境
* 出现libnvinfer.so, libnvinfer_plugin.so: cannot open shared object file 问题
* [Ubuntu22.04 下配置CUDA、cudnn和TensorRT环境_importerror: libnvinfer.so.8: cannot open shared o-CSDN博客](https://blog.csdn.net/weixin_60864335/article/details/126671341?spm=1001.2014.3001.5506) 
* [配置cuda和cudnn出现 libcudnn.so.8 is not a symbolic link问题_symbol _zn15tracebackloggerc1epkc version libcudnn-CSDN博客](https://blog.csdn.net/weixin_44609958/article/details/134349499) 



#### 点云标注annotate使用

```shell
1. roslaunch annotate demo.launch bag:="/usr/data/kitti_drive/kitti_drive2.bag --pause-topics /kitti/velo/pointcloud"
```



#### CMakeLists.txt编写

CMakeLists.txt是一个用于配置CMake项目的文本文件，它包含了一系列的指令和变量，用于指定项目的源文件、目标文件、依赖关系、编译选项、安装路径等信息。CMakeLists.txt的配置可以分为以下几个部分：

- 基础配置：这部分是对项目的基本信息进行说明，包括项目的名称、版本、语言、编译器、全局宏定义和头文件路径等。一般使用`project`、`set`、`add_definitions`、`include_directories`等指令进行配置。
- 编译目标文件：这部分是指定项目要生成的目标文件的类型、名称、源文件和链接库等。目标文件的类型一般有静态库、动态库和可执行文件，分别使用`add_library`、`add_executable`等指令进行创建，并使用`target_link_libraries`指令进行链接操作。
- 安装和打包：这部分是指定项目在执行安装和打包时，需要安装或打包的内容和目标路径等。一般使用`install`指令进行安装配置，使用`CPack`模块进行打包配置。

[如果你想了解更多关于CMakeLists.txt的配置的细节和示例，你可以参考以下的网页](https://zhuanlan.zhihu.com/p/371257515)[1](https://zhuanlan.zhihu.com/p/371257515)[2](https://blog.csdn.net/qq_38410730/article/details/102477162)[3](https://bing.com/search?q=CMakeLists.txt%E7%9A%84%E9%85%8D%E7%BD%AE)[4](https://zhuanlan.zhihu.com/p/648590071)[5](https://blog.csdn.net/u011086209/article/details/90036153)[6](https://zhuanlan.zhihu.com/p/587558418)。



* 使用 Eigen3 库和 yaml-cpp 库时，可以按照以下方式配置 CMakeLists.txt 文件： 

```cmake
# 基础配置
cmake_minimum_required(VERSION 3.10) # 指定CMake的最低版本要求
project(track) # 指定项目的名称
set(CMAKE_CXX_STANDARD 11) # 指定C++的标准版本
set(CMAKE_BUILD_TYPE Release) # 指定编译类型为Release模式

# 编译目标文件
add_executable(track track.cpp) # 指定要生成的可执行文件的名称和源文件
find_package(Eigen3 REQUIRED) # 查找Eigen3库是否安装
find_package(yaml-cpp REQUIRED) # 查找yaml-cpp库是否安装
include_directories(${EIGEN3_INCLUDE_DIR}) # 添加Eigen3的头文件路径
include_directories(${YAML_CPP_INCLUDE_DIR}) # 添加yaml-cpp的头文件路径
target_link_libraries(track ${YAML_CPP_LIBRARIES}) # 链接yaml-cpp的库文件

# 安装和打包
install(TARGETS track DESTINATION bin) # 指定可执行文件的安装路径为bin目录
set(CPACK_PACKAGE_NAME "track") # 指定打包的项目名称为track
set(CPACK_PACKAGE_VERSION "1.0.0") # 指定打包的项目版本为1.0.0
include(CPack) # 包含CPack模块
```



#### message::filter中的bind函数报错

* 在调用 `registerCallback` 函数时，回调函数的参数顺序与 `SyncPolicyT` 的模板参数顺序匹配。根据你提供的回调函数定义，参数顺序是正确的。 



#### 输出重定向

* rostopic echo > a.txt



#### 显示文本前几行

* cat a.txt | head -n 10



## ROS&&CV

#### 句柄

主要用于管理与ROS系统中各种资源的交互，充当与ROS节点、话题、服务参数之间通信和操作的接口

1. 节点句柄

   ```c
   //ROS节点与其他部分(话题、服务、参数)进行通信主要接口
   ros::NodeHandle nh;  // 创建节点句柄
   ros::Publisher pub = nh.advertise<std_msgs::String>("my_topic", 10);
   ```

2. 私有节点句柄

   ```c
   //是一个节点句柄的衍生，用于节点内部创建私有通信接口
   ros::NodeHandle private_nh("~");  // 创建私有节点句柄
   int my_param;
   private_nh.param("my_parameter", my_param, 42);
   ```


3. 其他句柄
   * getNodeHandle：获取一个节点的全局节点句柄
   * getPrivateNodeHandle：获取一个节点的私有节点句柄
   * getMTPrivateNodeHandle：多线程环境中创建私有节点句柄



#### opencv 4.6安装配置

[ubuntu下opencv4.6.0环境安装编译_prefix=/usr/local exec_prefix=${prefix} includedir-CSDN博客](https://blog.csdn.net/gentleman1358/article/details/126955032) 



#### ubuntu20.04安装libgtk-3遇到的问题

* 解决方法：更换源，换为aliyun



#### opencv contrib cmake编译时下载文件出错

* 解决方法：将cmake中的download换为国内
* [opencv contrib cmake编译时下载文件出错_cmake下载失败-CSDN博客](https://blog.csdn.net/qq_41061370/article/details/129178809?spm=1001.2014.3001.5506) 
* 第二个解决办法就是将CMakeLists里面要下载的文件下载下来，并且重新修改目录



#### ROS通信方式

1. 话题通信
2. 服务通信
3. 参数服务器



#### rosbag如何截取片段

```shell
1. rosbag play your_bag_file.bag
2. rosbag info your_bag_file.bag
3. rosbag filter your_bag_file.bag your_output_file.bag "yourstart_time <= t.to_sec() <= yourend_time"
​```
```



#### roslaunch

launch文件

> * node pkg : 对应功能包的名称
> * type：节点对应的可执行文件名
> * name：运行时显示的节点名称



#### ros启动多个launch文件

* 写shell脚本
* 创建一个组合launch文件

你可以创建一个新的 launch 文件，用来同时启动多个其他的 launch 文件。在这个组合的 launch 文件中，你可以使用 `<include>` 标签来引用其他的 launch 文件。例如：

```xml
<launch>
  <include file="$(find yourpackage_name)/launch/launch_file1.launch"/>
  <include file="$(find yourpackage_name)/launch/launch_file2.launch"/>
</launch>
```

然后，你可以使用 `roslaunch` 命令来启动这个组合的 launch 文件：

```shell
roslaunch package_name combined_launch_file.launch


  ros-noetic-jsk-rviz-plugins ros-noetic-rtabmap-demos
  ros-noetic-rtabmap-examples ros-noetic-rtabmap-launch
  ros-noetic-rtabmap-legacy ros-noetic-rtabmap-ros
  ros-noetic-rtabmap-rviz-plugins ros-noetic-rviz ros-noetic-rviz-imu-plugin
```





#### ros::spin()

主要作用是使节点保持运行，处理ROS消息和回调函数。

在`ros::spin()`开始运行后，节点会保持活动状态，处理消息，并执行回调函数，直到节点被显式地关闭或终止。通常，`ros::spin()`函数在节点的`main`函数中调用，以确保节点持续运行并处理消息，直到用户决定关闭节点。



#### tf::TransformListener

```shell
1. 允许ROS节点订阅和监听来自tf广播器（tf broadcaster）发布的坐标变换信息。它可以接收并缓存来自不同坐标系的变换关系，并提供查询接口，以便在节点中获取不同坐标系之间的变换关系
2. lookupTransform() 方法用于查询两个坐标系之间的变换关系
3. 可以通过订阅frame_id来查询tf

例如：

tf::StampedTransform
ROSRangeVisionFusionApp::FindTransform(const std::string &in_target_frame, const std::string &in_source_frame)
{
  tf::StampedTransform transform;

  ROS_INFO("%s - > %s", in_source_frame.c_str(), in_target_frame.c_str());
  camera_lidar_tf_ok_ = false;
  try
  {
    transform_listener_->lookupTransform(in_target_frame, in_source_frame, ros::Time(0), transform);
    camera_lidar_tf_ok_ = true;
    ROS_INFO("[%s] Camera-Lidar TF obtained", __APP_NAME__);
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("[%s] %s", __APP_NAME__, ex.what());
  }

  return transform;
}
```



#### tf::stampedTranform和tf::Transform区别

* `tf::StampedTransform`包含了时间戳信息，用于表示动态的坐标系变换，而`tf::Transform`则表示静态的坐标系变换 

  

#### jsk_recognition_utils::Cube

* 用于表示三维空间中的立方体（cube）

```c++
namespace jsk_recognition_utils
{
  class Cube
  {
  public:
    typedef boost::shared_ptr<Cube> Ptr;
    typedef boost::shared_ptr<const Cube> ConstPtr;

    // 构造函数，通过传递立方体的中心坐标、尺寸和姿态来创建 Cube 对象
    Cube(const Eigen::Vector3d& center, const Eigen::Vector3d& dimensions, const Eigen::Quaterniond& orientation);

    // 获取立方体的中心坐标
    const Eigen::Vector3d& getCentroid() const;

    // 获取立方体的尺寸
    const Eigen::Vector3d& getDimensions() const;

    // 获取立方体的姿态（四元数表示）
    const Eigen::Quaterniond& getOrientation() const;

    // ...
    // 其他成员函数和操作符重载
  };
}
```



#### Eigen::Affine3f

* 类的实例通常用于表示三维空间中的仿射变换，包括平移、旋转、缩放等操作



#### Eigen::MatrixXd

* 用于表示动态大小的矩阵，其中的元素可以是任意数据类型。

* 可以用于表示任意nxn的矩阵，以及运算

* Eigen::VectorXd只能表示一维动态数组

  ```cpp
  int main() {
      // 创建一个3x3的MatrixXd矩阵，元素类型为double
      Eigen::MatrixXd matrix(3, 3);
  
      // 初始化矩阵
      matrix << 1, 2, 3,
                4, 5, 6,
                7, 8, 9;
  
      // 输出矩阵
      std::cout << "Matrix:\n" << matrix << "\n";
  
      // 进行矩阵运算，例如矩阵乘法
      Eigen::MatrixXd result = matrix * matrix;
  
      // 输出运算结果
      std::cout << "Result:\n" << result << "\n";
  
      return 0;
  }
  ```







#### visualization_msgs::Marker

ROS中用于可视化目的的消息类型之一，通常用于在可视化工具中绘制3D场景中的各种标记，如点、线、箭头、文本等；这些标记可以用于显示传感器数据、路径规划、物体追踪以及其他机器人感知和控制任务。

> 1. **header**: 用于指定消息的时间戳和坐标系信息。
> 2. **ns (namespace)**: 命名空间，用于将不同标记进行分组，有助于组织可视化元素。
> 3. **id**: 每个标记的唯一标识符，用于区分同一命名空间中的不同标记。
> 4. **type**: 标记的类型，可以是诸如点、线、箭头、立方体、球体等不同的标记类型。
> 5. **action**: 指定如何处理标记，可以是添加、修改、删除等。
> 6. **pose**: 标记的姿态信息，通常指定了标记的位置和方向。
> 7. **scale**: 缩放因子，用于调整标记的大小。
> 8. **color**: 标记的颜色，通常使用RGBA格式来指定颜色和透明度。
> 9. **lifetime**: 标记的寿命，指定了标记在场景中存在的时间。
> 10. **frame_locked**: 一个布尔值，用于指定标记的姿态是否与坐标系锁定。
> 11. **points**: 对于某些标记类型，如线条和箭头，这个字段包含了标记的几何形状的定义。
> 12. **text**: 如果标记类型是文本，这个字段包含了文本的内容。



#### message_filters::Subscriber

> * 用于ROS节点中订阅消息并提供时间同步的功能；当消息到达消息过滤器的时候，不会立即输出，而是在稍后的时间点里满足一定条件下输出；
> * 本质是消息缓冲区来存储来自不同消息话题的消息。每个缓冲区用于存储一个消息话题的消息队列。
> * 当 `message_filters::Synchronizer` 接收到来自不同消息话题的消息时，它会记录每个消息的时间戳。时间戳被用来确定消息之间的时间差。
> * 为每个消息话题设置一个时间窗口。时间窗口是一个时间间隔，例如10毫秒。如果消息的时间戳在时间窗口内，它们将被认为是同步的。



```c++
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu>

void callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::ImuConstPtr& imu_msg) {
    // 在这里执行时间同步的处理逻辑
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "synced_node");
    ros::NodeHandle nh;

    // 创建两个消息订阅者
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/image_topic", 1);
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/imu_topic", 1);

    // 定义时间同步策略
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Imu> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, imu_sub);

    // 设置回调函数
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
    return 0;
}

```



#### message_filters中的时间同步问题

* 对齐时间戳有两种方式，一种是时间戳完全对齐 ：**ExactTime Policy** ，另一种是时间戳相近：**ApproximateTime Policy**

* 两个传感器时间戳必须格式一致，否则进入不了回调函数
* 例如lslidar与相机调试中的时间同步，lslidar时间戳不更新，或者相机时间戳中断



#### geometry_msgs::Twist

* 用于表示机器人的线速度和角速度的消息类型

```c++
//成员变量
geometry_msgs/Vector3 linear;
geometry_msgs/Vector3 angular;

//geometry_msgs::Vector3
float64 x
float64 y
float64 z
```



#### visualization_msgs::Marker 

* ROS（Robot Operating System）中的一个消息类型，用于在可视化工具中显示各种类型的标记（Marker） 

```shell
header：标记的头部信息，包括时间戳和坐标系。
ns：命名空间，用于将标记分组，以便于管理和筛选。
id：标记的唯一标识符，在同一命名空间中必须唯一。
type：标记的类型，可以是点、线、箭头、立方体、球体等。
action：标记的操作类型，可以是添加、修改、删除等。
pose：标记的位置和姿态信息。
scale：标记的大小，用于调整标记的尺寸。
color：标记的颜色，可以是 RGBA 格式的颜色值。
points：用于线条和多边形标记的点集。
text：用于文本标记的文本内容。
lifetime：标记的生命周期，控制标记在可视化工具中的显示时间。
```





#### IplImage

* IplImage 是 OpenCV 旧版本（OpenCV 2及以下）中使用的图像数据结构
* 从 OpenCV 3.0 版本开始，推荐使用更先进的 cv::Mat 数据结构来替代 IplImage



#### cv::FileStorage

OpenCV库中的一个类，用于在磁盘上读写和处理文件，主要用于配置参数文件

```c++
//1
cv::FileStorage fs("config.yml", cv::FileStorage::WRITE);
if (fs.isOpened()) {
    fs << "param1" << 42;
    fs << "param2" << 3.14;
    fs << "param3" << "Hello, OpenCV";
    fs.release(); // 保存并关闭文件
}

cv::FileStorage fs2("config.yml", cv::FileStorage::READ);
if (fs2.isOpened()) {
    int param1;
    double param2;
    std::string param3;
    fs2["param1"] >> param1;
    fs2["param2"] >> param2;
    fs2["param3"] >> param3;
    fs2.release(); // 关闭文件

    // 使用加载的参数
    std::cout << "param1: " << param1 << std::endl;
    std::cout << "param2: " << param2 << std::endl;
    std::cout << "param3: " << param3 << std::endl;
}

//2
std::string str_initial_file = pkg_loc + "/cfg/initial_params.yaml";

cv::FileStorage fs(str_initial_file, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML);
if (!fs.isOpened())
{
    std::cout << " [ " + str_initial_file + " ] 文件打开失败" << std::endl;
    return 0;
}

i_params.camera_topic = std::string(fs["image_topic"]);
i_params.lidar_topic = std::string(fs["pointcloud_topic"]);
i_params.fisheye_model = int(fs["distortion_model"]);
i_params.lidar_ring_count = fs["scan_line"];
i_params.grid_size = std::make_pair(int(fs["chessboard"]["length"]), int(fs["chessboard"]["width"]));
i_params.square_length = fs["chessboard"]["grid_size"];
i_params.board_dimension = std::make_pair(int(fs["board_dimension"]["length"]), int(fs["board_dimension"]["width"]));
i_params.cb_translation_error = std::make_pair(int(fs["translation_error"]["length"]), int(fs["translation_error"]["width"]));
fs["camera_matrix"] >> i_params.cameramat;
i_params.distcoeff_num = 5;
fs["distortion_coefficients"] >> i_params.distcoeff;

i_params.image_size = std::make_pair(int(fs["image_pixel"]["width"]), int(fs["image_pixel"]["height"]));

//3
void Write_XML(){
	cv::FileStorage fs("test.yml", cv::FileStorage::WRITE);
	fs << "frameCount" << 5;
	time_t rawTime;
	time(&rawTime);
	fs << "calibrationDate" << asctime(localtime(&rawTime));
 
	cv::Mat cameraMaterix = (cv::Mat_<double>(3, 3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);
	cv::Mat distCoeffs = (
		cv::Mat_<double>(5, 1)
		<< 0.1, 0.01, -0.001, 0, 0
		);
	fs << "cameraMatrix" << cameraMaterix << "distCoeffs" << distCoeffs;
 
	fs << "features" << "[";
	for (int i = 0; i < 3; i++){
		int x = rand() % 640;
		int y = rand() % 480;
		uchar lbp = rand() % 256;
		fs << "{:" << "x" << x << "y" << y << "lbp" << "[:";
		for (int j = 0; j < 8; j++){
			fs << ((lbp >> j) & 1);
		}
		fs << "]" << "}";
	}
	fs << "]";
	fs.release();
}
 
void Read_XML(){
	cv::FileStorage fs("test.yml", cv::FileStorage::READ);
	
	// 直接进行数据转换
	int frameCount = (int)fs["frameCount"];
 
	// 使用操作符 >> 转换
	std::string date;
	fs["calibrationDate"] >> date;
	cv::Mat cameraMatrix, distCoeffs;
	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;
 
	std::cout << "frameCount:" << frameCount << std::endl
		<< "calibration date:" << date << std::endl
		<< "camera matrix:	" << cameraMatrix << std::endl
		<< "distortion coeffs: " << distCoeffs << std::endl;
 
	cv::FileNode features = fs["features"];
	cv::FileNodeIterator it = features.begin(), it_end = features.end();
	int idx = 0;
	std::vector<uchar> lbpval;
	 // 遍历一个sequence
	for (; it != it_end; ++it, idx++){
		std::cout << "feature #" << idx << ":";
		std::cout << "x=" << (int)(*it)["x"]
			<< ", y=" << (int)(*it)["y"]
			<< ", lbp:(";
		// 使用cv::FileNode >> std::vector读取数组
		(*it)["lbp"] >> lbpval;
		for (int i = 0; i < (int)lbpval.size(); i++){
			std::cout << " " << (int)lbpval[i];
		}
		std::cout << ")" << std::endl;
	}
	fs.release();
}
```

<img src="/home/gmm/.config/Typora/typora-user-images/image-20231026101638281.png" alt="image-20231026101638281" style="zoom:50%;" />



#### cv::bundingRect

* OpenCV 中的一个函数，用于计算给定轮廓的最小外接矩形

```c++
cv::Rect cv::boundingRect(
    InputArray points  // 输入的轮廓点集
);
```

* points：输入的轮廓点集，可以是一个表示轮廓的 std::vector<cv::Point> 或 cv::Mat
* 函数返回一个 cv::Rect 对象，表示计算得到的最小外接矩形



#### cv_bridge::CvImagePtr

* 用于在ROS和OpenCV之间的图像数据转换和传递

```c++
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // 访问OpenCV图像数据
        cv::Mat image = cv_ptr->image;

        // 在这里进行图像处理
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_processing_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/image_topic", 1, imageCallback);

    ros::spin();
    return 0;
}
```



#### cv::Mat

* 创建cv::Mat对象

```c++
cv::Mat image(rows, cols, type);
cv::Mat image(rows, cols, type, data);
```

* 图像读取与显示

```c++
cv::Mat image = cv::imread("image.jpg");
cv::imshow("Window Name", image);
```

* 图像保存

```c++
cv::imwrite("output.jpg", image);
```

* 图像剪裁与复制

```c++
cv::Mat subImage = image(cv::Rect(x, y, width, height));
cv::Mat copy = image.clone();
```

* 绘制图像

```c
cv::putText(image, "Hello, OpenCV", cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, fontSize, cv::Scalar(255, 0, 0), thickness);
cv::line(image, startPoint, endPoint, cv::Scalar(0, 255, 0), thickness);
cv::rectangle(image, topLeft, bottomRight, cv::Scalar(0, 0, 255), thickness);
```

* 赋值方式

> Mat a(3, 3, CV_8UC1);
>
> Mat b(Size(4, 4), CV_8UC1);
>
> Mat c0(5, 5, CV_8UC1, Scalar(4, 5, 6))
>
> Mat d = (cv::Mat_<int>(1, 5) << 1, 2, 3, 4, 5)



#### cv::Size2f

* 用于表示二维大小（尺寸），其中的尺寸值是以浮点数表示

```c++
//构造函数
cv::Size2f::Size2f();
cv::Size2f::Size2f(float width, float height);
    
//用例
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;
int main()
{
    // 创建一个矩形大小，宽度为 10.5，高度为 20.3
    Size2f sz(10.5, 20.3);
    // 输出它的宽度、高度和面积
    cout << "width: " << sz.width << endl;
    cout << "height: " << sz.height << endl;
    cout << "area: " << sz.area() << endl;
    return 0;
}

```



#### cv::Rect

```c++
template<typename _Tp> class Rect_
{
public:
    typedef _Tp value_type;

    //! default constructor
    Rect_();
    Rect_(_Tp _x, _Tp _y, _Tp _width, _Tp _height);
    Rect_(const Rect_& r);
    Rect_(Rect_&& r) CV_NOEXCEPT;
    Rect_(const Point_<_Tp>& org, const Size_<_Tp>& sz);
    Rect_(const Point_<_Tp>& pt1, const Point_<_Tp>& pt2);

    Rect_& operator = ( const Rect_& r );
    Rect_& operator = ( Rect_&& r ) CV_NOEXCEPT;
    //! the top-left corner
    Point_<_Tp> tl() const;
    //! the bottom-right corner
    Point_<_Tp> br() const;

    //! size (width, height) of the rectangle
    Size_<_Tp> size() const;
    //! area (width*height) of the rectangle
    _Tp area() const;
    //! true if empty
    bool empty() const;

    //! conversion to another data type
    template<typename _Tp2> operator Rect_<_Tp2>() const;

    //! checks whether the rectangle contains the point
    bool contains(const Point_<_Tp>& pt) const;

    _Tp x; //!< x coordinate of the top-left corner
    _Tp y; //!< y coordinate of the top-left corner
    _Tp width; //!< width of the rectangle
    _Tp height; //!< height of the rectangle
};

typedef Rect_<int> Rect2i;
typedef Rect_<float> Rect2f;
typedef Rect_<double> Rect2d;
typedef Rect2i Rect;
```



#### cv::line

* 用于绘制一条线段，连接两个点

```c++
cv::line(image, start_point, end_point, color, thickness);
```





#### cv::resize

* OpenCV 中用于调整图像大小的函数。它允许您将图像缩放到指定的尺寸或按比例进行调整。

```c++
void cv::resize(
    InputArray src,         // 输入图像
    OutputArray dst,        // 输出图像
    Size dsize,             // 目标尺寸
    double fx = 0,          // 水平方向缩放比例
    double fy = 0,          // 垂直方向缩放比例
    int interpolation = INTER_LINEAR  // 插值方法
);
```



#### cv::findChessboardCorners

* 用于检测棋盘格模式的角点

```c++
bool cv::findChessboardCorners(
    InputArray image,          // 输入图像
    Size patternSize,          // 棋盘格内角点的行列数
    OutputArray corners,       // 输出的角点坐标
    int flags = CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE  // 检测参数标志
);
```

```c++
cv::Mat image = cv::imread("chessboard.jpg", cv::IMREAD_GRAYSCALE);
// 读取灰度图像

cv::Size patternSize(9, 6);
std::vector<cv::Point2f> corners;
bool found = cv::findChessboardCorners(image, patternSize, corners);

if (found) {
    cv::drawChessboardCorners(image, patternSize, corners, found);
    cv::imshow("Chessboard", image);
    cv::waitKey(0);
}
```



#### cv::projectPoints 

* 用于将三维点云投影到相机平面上 

```c++
void cv::projectPoints(
    InputArray objectPoints,       // 输入的三维点云坐标
    InputArray rvec,               // 旋转向量
    InputArray tvec,               // 平移向量
    InputArray cameraMatrix,       // 相机内参矩阵
    InputArray distCoeffs,         // 畸变系数
    OutputArray imagePoints,       // 输出的投影点坐标
    OutputArray jacobian = noArray(), // 输出的雅克比矩阵（可选）
    double aspectRatio = 0        // 输出图像的像素宽高比（可选）
);
```

```c++
std::vector<cv::Point3f> objectPoints;
// 假设 objectPoints 为一组三维点云坐标

cv::Mat rvec;  // 旋转向量
cv::Mat tvec;  // 平移向量
// 假设 rvec 和 tvec 为相机的旋转和平移姿态

cv::Mat cameraMatrix;
// 假设 cameraMatrix 为相机的内参矩阵

cv::Mat distCoeffs;
// 假设 distCoeffs 为相机的畸变系数

std::vector<cv::Point2f> imagePoints;
// 用于存储投影点坐标

cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, imagePoints);
```



#### cv::solvePnP 

* 用于解算相机的姿态（旋转和平移） 

```c++
bool cv::solvePnP(
    InputArray objectPoints,      // 物体坐标点的数组或向量
    InputArray imagePoints,       // 图像坐标点的数组或向量
    InputArray cameraMatrix,      // 相机内参矩阵
    InputArray distCoeffs,        // 畸变系数
    OutputArray rvec,             // 输出的旋转向量
    OutputArray tvec,             // 输出的平移向量
    bool useExtrinsicGuess = false,// 是否使用外部姿态作为初始猜测（可选）
    int flags = SOLVEPNP_ITERATIVE // 解算方法的标志（可选）
);
```

```c++
std::vector<cv::Point3f> objectPoints;
// 假设 objectPoints 为一组物体坐标点

std::vector<cv::Point2f> imagePoints;
// 假设 imagePoints 为一组图像坐标点

cv::Mat cameraMatrix;
// 假设 cameraMatrix 为相机的内参矩阵

cv::Mat distCoeffs;
// 假设 distCoeffs 为相机的畸变系数

cv::Mat rvec;  // 输出的旋转向量
cv::Mat tvec;  // 输出的平移向量

cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
```



#### cv::Rodrigues 

* 将旋转向量转换为旋转矩阵，或将旋转矩阵转换为旋转向量 

```c
void cv::Rodrigues(
    InputArray src,    // 输入的旋转向量或旋转矩阵
    OutputArray dst,   // 输出的旋转矩阵或旋转向量
    OutputArray jacobian = noArray() // 可选的输出雅可比矩阵
);
```

```c
cv::Mat rvec;
// 假设 rvec 为输入的旋转向量

cv::Mat R;  // 输出的旋转矩阵

cv::Rodrigues(rvec, R);
```



#### cv::circle 

* 用于在图像上绘制圆的函数之一

```c++
void cv::circle(
    InputOutputArray img,     // 输入输出图像
    Point center,             // 圆心坐标
    int radius,               // 圆的半径
    const Scalar& color,      // 圆的颜色
    int thickness = 1,        // 圆的线条粗细（可选，默认为 1）
    int lineType = LINE_8,    // 线条类型（可选，默认为 8 连通线）
    int shift = 0             // 坐标点的小数位数（可选，默认为 0）
);
```

```c++
cv::Mat img;
// 假设 img 是输入图像

cv::Point center(100, 100);
// 假设圆心坐标为 (100, 100)

int radius = 50;
// 假设圆的半径为 50

cv::Scalar color(0, 0, 255);
// 假设圆的颜色为红色

int thickness = 2;
// 假设线条粗细为 2

cv::circle(img, center, radius, color, thickness);
```



#### cv::cvtColor

* 用于图像颜色空间的转换

```c++
void cv::cvtColor(
    InputArray src,           // 输入图像
    OutputArray dst,          // 输出图像
    int code,                 // 颜色空间转换代码
    int dstCn = 0             // 输出图像的通道数，如果为 0，则根据代码自动确定
);
```



#### cv::copyMakeBorder

* 用于对图像进行边界扩展（border extension）操作，在图像的边界添加额外的像素值

```c++
void cv::copyMakeBorder(
    InputArray src,              // 输入图像
    OutputArray dst,             // 输出图像
    int top, int bottom,         // 上下边界的大小
    int left, int right,         // 左右边界的大小
    int borderType,              // 边界类型
    const Scalar& value = Scalar()  // 扩展的像素值
);
```



#### cv::RotatedRect

* 用于表示旋转矩形 

```c++
//构造函数
cv::RotatedRect::RotatedRect();
cv::RotatedRect::RotatedRect(const cv::Point2f& center, const cv::Size2f& size, float angle);

//用例
#include <opencv2/opencv.hpp>
using namespace cv;
int main()
{
    // 创建一个 200x200 的图像，背景为黑色
    Mat test_image(200, 200, CV_8UC3, Scalar(0));
    // 创建一个旋转矩形，中心点为 (100,100)，边长为 (100,50)，旋转角度为 30 度
    RotatedRect rRect = RotatedRect(Point2f(100, 100), Size2f(100, 50), 30);
    // 获取旋转矩形的四个顶点
    Point2f vertices[4];
    rRect.points(vertices);
    // 在图像上绘制旋转矩形的边框，颜色为绿色
    for (int i = 0; i < 4; i++)
        line(test_image, vertices[i], vertices[(i + 1) % 4], Scalar(0, 255, 0), 2);
    // 获取旋转矩形的包围矩形
    Rect brect = rRect.boundingRect();
    // 在图像上绘制包围矩形的边框，颜色为红色
    rectangle(test_image, brect, Scalar(0, 0, 255), 2);
    // 显示图像
    imshow("rectangles", test_image);
    waitKey(0);
    return 0;
}

```



#### 旋转平移矩阵

* 其中dx,dy为target的位置相对于起始点位置的差
* 旋转平移矩阵*起始点坐标 = target目标的位置



#### 相机参数

* 内参K矩阵

```c
| f_x   0   c_x |
|  0   f_y  c_y |
|  0    0    1  |
  
# `f_x` 和 `f_y` 表示焦距，通常以像素为单位。它们分别表示在图像的 x 和 y 方向上的焦距。
# `c_x` 和 `c_y` 是主点（Principal Point）的坐标，表示成像平面的中心点在图像坐标系中的坐标。通常以像素为单位。
```

* 相机失真系数D

```c
 径向畸变参数（Radial Distortion Coefficients）：
     k1: 一阶径向畸变参数
     k2: 二阶径向畸变参数
     k3: 三阶径向畸变参数

  这些参数描述了光线离开相机光轴时的弯曲程度。一阶径向畸变（k1）通常是最主要的，它表示从图像中心向外的距离越远，畸变越明显。二阶和三阶径向畸变参数用于更精确地建模镜头的畸变。

  切向畸变参数（Tangential Distortion Coefficients）：
      p1: 第一个切向畸变参数
      p2: 第二个切向畸变参数

  这些参数描述了图像平面不是正好与光线平行时引入的畸变。切向畸变参数主要用于校正图像中的倾斜或非正交畸变。
```



#### Kitti数据集坐标系

[KITTI数据集解析和可视化_kitti数据集可视化-CSDN博客](https://blog.csdn.net/zyw2002/article/details/127395975?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522170255525216800215058003%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=170255525216800215058003&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-127395975-null-null.142^v96^pc_search_result_base5&utm_term=kitti%E6%95%B0%E6%8D%AE%E9%9B%86&spm=1018.2226.3001.4187) 

[KITTI数据集--参数-CSDN博客](https://blog.csdn.net/cuichuanchen3307/article/details/80596689) 

[Kitti数据集标签中yaw角在不同坐标系的转换_kitti 角度-CSDN博客](https://blog.csdn.net/ppppiu/article/details/121905961?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522170355815616800225567015%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=170355815616800225567015&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-3-121905961-null-null.142^v96^pc_search_result_base5&utm_term=yaw%E8%A7%92%E5%BA%A6%E6%AD%A3%E8%B4%9F&spm=1018.2226.3001.4187) 



#### 相机坐标系转换关系

相机总共四个坐标系：世界坐标系，相机坐标系，图像坐标系，像素坐标系

* 世界坐标系到相机坐标系

$$
P_c = \left[
\matrix{
  R & T\\
  0 & 1\\
}
\right].P_w
$$

* 相机坐标系到图像坐标系

$$
Z_c .\left[
\matrix{
  x\\
  y\\
  1
}
\right]= \left[
\matrix{
 K|0
}
\right].\left[
\matrix{
  X_c\\
  Y_c\\
  Z_c\\
  1
}
\right]
$$

* 图像坐标系到像素坐标系

$$
\left[
\matrix{
  u\\
  v\\
  1
}
\right]= \left[
\matrix{
 \frac{1}{d_x} & 0 & u_0 \\
  0 & \frac{1}{d_y} & v_0 \\
  0 & 0 & 1
}
\right].\left[
\matrix{
  x\\
  y\\
  1
}
\right]
$$



#### 雷达坐标系(velo_link)

```shell
      x(车辆的朝向)
      |
      |
y—— ——z(朝向外)
```



#### 相机坐标系

```shell
	z(车辆朝向)
	|
	|
	y(朝向里)—— ——x
```



#### 图像坐标系

```shell
o —— ——> x(width)
|
|
y(height)
```



#### MOT标注数据格式

* 2D图像

```shell
frame, id, x, y, width, height, conf(置信度), -1, -1, -1
```



#### GPS转换为东北天坐标

```c++
//读取
//read gps data get the longitude latitude
std::string gpspath = path + "/oxts/"+file+".txt";
cout<<gpspath<<endl;
std::ifstream gps(gpspath);
std::vector<std::string> gpsdata;
if (gps) {
    boost::char_separator<char> sep_line { "\n" };
    std::stringstream buffer;
    buffer << gps.rdbuf();
    std::string contents(buffer.str());
    tokenizer tok_line(contents, sep_line);
    std::vector<std::string> lines(tok_line.begin(), tok_line.end());
    gpsdata = lines;
    int size = gpsdata.size();

    cout<<size<<" "<< lidarname.size()<<endl;
    if(size != lidarname.size()){
        cout<<"file number wrong!"<<endl;
        std::abort();
    }
}

//........
//.........

//转换
//get the gps data and get the tranformation matrix
tokenizer tokn(gpsdata[frame], sep);
vector<string> temp_sep(tokn.begin(), tokn.end());

double UTME,UTMN;
double latitude = stringToNum<double>(temp_sep[0]);
double longitude = stringToNum<double>(temp_sep[1]);
double heading = stringToNum<double>(temp_sep[5]) - 90 * M_PI/180;
LonLat2UTM(longitude, latitude, UTME, UTMN);
//调用一个函数 LonLat2UTM，将经度和纬度值作为参数传递给该函数，并将转换后的UTM坐标存储在 UTME 和 UTMN 变量中。

```





## 算法相关

#### 深度学习

* 卷积

```shell
1. 将卷积核映射到图像上，然后逐个相乘累加的运算求和
2. 注意卷积操作的位置，右边最后一列从下往上数对应映射第一行卷积参数
3. nxn的图像采用fxf的卷积和得到(n-f+1)x(n-f+1)图像
```

![Snipaste_2023-12-20_15-23-15](D:\xiaoxin_Pro16\桌面\文章图片\Snipaste_2023-12-20_15-23-15.png)

* 三维卷积

```shell
1. 将卷积核对应相应的输入
2. 将每个格子对应的参数相乘然后累加
```

* 1x1卷积核

```shell
1. 将输入的通道数改变
2. 例如一个28x28x192的输入想要变为28x28x32的输出，可以采用32个1x1x192的卷积核
```

* 填充

```shell
1.为了使边缘信息不丢失
2.为了使卷积之后图像仍然为原来大小
3.若卷积之后仍为原来大小则填充层为p = (f-1)/2
```

* 步长

```shell
1.步长s影响最后输出为：(n+2p-f)/s + 1
2.如果不能除整，则向下取整
```

* n个卷积核输出

```
加入有一个6x6x3的张量输入，现在有两个3x3x3的卷积核，那么输出为4x4x2的张量
```

![Snipaste_2023-12-20_15-06-47](D:\xiaoxin_Pro16\桌面\文章图片\Snipaste_2023-12-20_15-06-47.png)



* 池化

```shell
1. 区域划分，然后取每个区域最大值;
2. 参数f(尺寸),s(步长);
3. 输出：(n-f)/s + 1，向下取整
4. 作用：改变张量的Nh和Nw
```



#### 卷积神经网络的函数拟合

* 卷积后输出

$$
Z^{[1]}=W^{[1]}a^{[0]}+b^{[1]}
$$

* $$
  a^{[0]}是输入的张量，W^{[1]}是卷积操作
  $$

  

* 加入Relu后

$$
a^{[1]}=g(Z^{[1]})
$$



#### 网络参数个数计算

![Snipaste_2023-12-26_21-16-52](D:\xiaoxin_Pro16\桌面\文章图片\Snipaste_2023-12-26_21-16-52.png)

![Snipaste_2023-12-26_21-17-54](D:\xiaoxin_Pro16\桌面\文章图片\Snipaste_2023-12-26_21-17-54.png)



#### AlexNet

* [AlexNet包含八层，前五层是卷积层，其中一些后面跟着最大池化层，最后三层是全连接层。](https://en.wikipedia.org/wiki/AlexNet) 
* [第一个在物体识别方面使用深度学习和卷积神经网络的架构，它改变了计算机视觉和机器学习的范式](https://paperswithcode.com/method/alexnet) 

![Snipaste_2023-12-20_16-12-35](D:\xiaoxin_Pro16\桌面\文章图片\Snipaste_2023-12-20_16-12-35.png)



#### VGGNet

* [VGGNet是深度学习和计算机视觉领域的一个重要的基准，它为后续的网络架构提供了启发和参考](https://pytorch.org/hub/pytorch_vision_vgg/)

![Snipaste_2023-12-20_16-21-03](D:\xiaoxin_Pro16\桌面\文章图片\Snipaste_2023-12-20_16-21-03.png)



#### ResNet

* 解决深度层次神经网络梯度/消失爆炸问题

![Snipaste_2023-12-20_16-38-58](D:\xiaoxin_Pro16\桌面\文章图片\Snipaste_2023-12-20_16-38-58.png)

![Snipaste_2023-12-20_16-36-12](D:\xiaoxin_Pro16\桌面\文章图片\Snipaste_2023-12-20_16-36-12.png)

![Snipaste_2023-12-20_20-32-00](D:\xiaoxin_Pro16\桌面\文章图片\Snipaste_2023-12-20_20-32-00.png)

#### Relu激活函数

```python
if input > 0:
  return input
else:
  return 0
```

* 计算简单，能够输出一个真正的零值 。这与 tanh 和 sigmoid 激活函数不同，后者学习近似于零输出，例如一个非常接近于零的值，但不是真正的零值。这意味着负输入可以输出真零值，允许神经网络中的隐层激活包含一个或多个真零值。这就是所谓的稀疏表示，是一个理想的性质，在表示学习，因为它可以加速学习和简化模型。
* RELU6

![Snipaste_2023-12-26_21-57-09](D:\xiaoxin_Pro16\桌面\文章图片\Snipaste_2023-12-26_21-57-09.png)



#### Inception 层

* 通过并行使用多个不同大小的卷积核和池化操作来捕捉不同尺度的图像特征。这种并行结构可以使网络在不同层次上具有多个感受野，从而提高了网络的表示能力。
* 增大感受野，而且还可以提高神经网络的鲁棒性 

![Snipaste_2023-12-26_20-20-59](D:\xiaoxin_Pro16\桌面\文章图片\Snipaste_2023-12-26_20-20-59.png)



#### MobileNet

* 一种轻量级的卷积神经网络，专门设计用于移动和嵌入式视觉应用。MobileNet的核心思想是使用深度可分离卷积（depthwise separable convolution），
* 将标准的卷积分解为一个深度卷积和一个逐点卷积，从而减少参数和计算量。

![Snipaste_2023-12-26_20-58-26](D:\xiaoxin_Pro16\桌面\文章图片\Snipaste_2023-12-26_20-58-26.png)

![Snipaste_2023-12-26_20-56-43](D:\xiaoxin_Pro16\桌面\文章图片\Snipaste_2023-12-26_20-56-43.png)

![Snipaste_2023-12-26_21-22-02](D:\xiaoxin_Pro16\桌面\文章图片\Snipaste_2023-12-26_21-22-02.png)



#### MobilenetV2

* 被广泛应用于移动设备和嵌入式系统中的图像分类、目标检测和图像分割等任务，为资源受限的场景提供了一种高效的解决方案
* 先用1x1卷积核升维到原来6倍，再用1x1卷积核降维

![Snipaste_2023-12-26_21-53-36](D:\xiaoxin_Pro16\桌面\文章图片\Snipaste_2023-12-26_21-53-36.png)

![Snipaste_2023-12-26_21-55-32](D:\xiaoxin_Pro16\桌面\文章图片\Snipaste_2023-12-26_21-55-32.png)

![Snipaste_2023-12-26_21-58-57](D:\xiaoxin_Pro16\桌面\文章图片\Snipaste_2023-12-26_21-58-57.png)



![Snipaste_2023-12-26_22-01-54](D:\xiaoxin_Pro16\桌面\文章图片\Snipaste_2023-12-26_22-01-54.png)

#### EfficientNet

* EfficientNet是一种高效的卷积神经网络架构，由Google Brain团队在2019年提出。它通过使用复合缩放方法来同时调整网络的深度、宽度和分辨率，以获得更好的性能和计算效率。





#### Sigmod激活函数

$$
sigmod(x) = 1/(1+exp^{-x})
$$



#### softmax激活函数

![Snipaste_2023-12-20_20-47-07](D:\xiaoxin_Pro16\桌面\文章图片\Snipaste_2023-12-20_20-47-07.png)



#### 卡尔曼滤波

* 先验估计

$$
\hat X^{-}_k=A\cdot\hat X_k+Bu_{k-1}
$$

* 后验估计

$$
\hat X_k=\hat X^{-}_k+K_k(Z_k-H\hat X^{-}_k)
$$

* 先验估计协方差

$$
P^{-}_k=A\cdot P_{k-1}\cdot A^{T} + Q
$$

* 后验估计协方差

$$
P_k=(I-K_kH)\cdot P^{-}_k
$$

* 新息

$$
Z_k-H\cdot \hat X^{-}_k
$$

* [卡尔曼滤波-卡尔曼滤波全篇讲解-CSDN博客](https://blog.csdn.net/weixin_43976737/article/details/119704221?spm=1001.2014.3001.5506) 



#### 目标检测中的损失量计算

![Snipaste_2023-12-27_21-33-42](D:\xiaoxin_Pro16\桌面\文章图片\Snipaste_2023-12-27_21-33-42.png)

* 其中y_hat是神经网络输出的结果，y是标签的结果



#### 目标检测中的滑动窗口

* 先训练一个能够检测是否为车辆的判别
* 然后逐个遍历



#### 全连接层转换为卷积层实现滑动窗口

![Snipaste_2023-12-27_21-54-20](D:\xiaoxin_Pro16\桌面\文章图片\Snipaste_2023-12-27_21-54-20.png)

![Snipaste_2023-12-27_22-02-36](D:\xiaoxin_Pro16\桌面\文章图片\Snipaste_2023-12-27_22-02-36.png)



#### yolo算法

* 将图像划分成网格
* 将之前的检测算法应用于网格之中
* 网格输出

![Snipaste_2024-01-02_20-21-52](D:\xiaoxin_Pro16\桌面\文章图片\Snipaste_2024-01-02_20-21-52.png)



#### 非极大值抑制NMS

* 算法可能对同一个目标重复检测
* 找到置信度最高的检测框，将其他与这个检测框有较高IoU的检测框进行抑制输出

![Snipaste_2024-01-02_20-37-33](D:\xiaoxin_Pro16\桌面\文章图片\Snipaste_2024-01-02_20-37-33.png)



#### AnchorBox

* 对于一个网格需要输出多个检测目标，则需要不同形状的anchorbox
* 然后比较哪个anchorbox的IoU最高

![Snipaste_2024-01-02_20-42-06](D:\xiaoxin_Pro16\桌面\文章图片\Snipaste_2024-01-02_20-42-06.png)



#### 模型部署

* [算法面试高频知识点：模型部署总结_牛客网 (nowcoder.com)](https://www.nowcoder.com/discuss/390262828702216192) 



#### 激光雷达目标检测

1. 体素分割->重新拼接->张量投影到二维平面->作一个类似于Poinpillars的2D伪图像
2. 降采样->升维
3. 类似SSD的检测头设置 ->设置IoU阈值



#### 感知融合

3D检测框信息投影到2D平面，计算IoU，如果超过一定阈值则进行匹配，这个匹配的检测框被赋予二维相机检测的属性



#### Autoware中的IMM-UKF-PDA算法

* 状态向量是五个维度（x,y,v,theta,w）
* UKF初始化，包括位置(x,y)初始化和航向角(theta)初始化
* 然后利用这些初始化向量去带入状态方程和量测方程生成一步预测
* 这时候再用PDA算法对每个目标的量测向量进行筛选，每个目标可能有多个可能相关的匹配
* 筛选合适的量测向量与前面的一步预测生成卡尔曼最优估计

![1702473860659](C:\Users\GMM\AppData\Local\Temp\1702473860659.png)



#### 跟踪算法

>UKF：本质是对非线性卡尔曼滤波的线性化，利用SigmaPoint去拟合；比如我用的是5维的状态向量分别是x,y,v,w,w'；通过SigmaPoint去拟合非线性，就得到11维度的SigmaPoint；再去求一步预测，进而求加权平均；再去求先验估计协方差和新息协方差；后验估计以及逐步递推；
>
>IMM：交互式多模型卡尔曼滤波；使用一种系统动态模型的卡尔曼滤波器对于多种运动状态的跟踪预测不匹配；比如我有两个状态对应两个滤波器，模型之间有一个2x2的转移矩阵；分别去求他们的权重，不断更新；再去加权求卡尔曼滤波；
>
>PDA算法：在实际的卡尔曼滤波中有预测的轨迹和我量测的轨迹，交杂在一起，这个算法的主要目的是判定轨迹关联，除了这个算法还有KNN算法，基于马氏距离的判定，JPDA, CJPDA。为每个有效量测计算一个概率，并结合新息计算出最优量测。



#### 多目标跟踪指标计算

* **MOTA(多目标跟踪准确度)**

假设有一个视频序列，共有10帧，每帧中有两个真实目标，用A和B表示；跟踪器检测到的目标用数字1,2,3,4表示。下表展示了每帧中的真实目标和检测目标的对应关系：

| 帧数 | 真实目标 | 检测目标 | FN   | FP   | IDSW |
| ---- | -------- | -------- | ---- | ---- | ---- |
| 1    | A, B     | 1, 2     | 1    | 0    | 0    |
| 2    | A, B     | 1, 2, 3  | 0    | 1    | 0    |
| 3    | A, B     | 1, 2     | 0    | 0    | 0    |
| 4    | A, B     | 1, 2     | 0    | 0    | 0    |
| 5    | A, B     | 2, 1     | 0    | 0    | 2    |
| 6    | A, B     | 2, 1     | 0    | 0    | 0    |
| 7    | A, B     | 1, 2     | 0    | 0    | 2    |
| 8    | A, B     | 1, 2     | 0    | 0    | 0    |
| 9    | A, B     | 1, 2, 4  | 0    | 1    | 0    |
| 10   | A, B     | 2        | 1    | 0    | 0    |
| 总计 | 20       | 20       | 2    | 2    | 4    |

从表中可以看出，跟踪器在第1帧和第10帧中漏报了目标A，即FN=2；在第2帧和第9帧中误报了目标3和4，即FP=2；在第5帧和第6帧中，目标A和B的ID发生了切换，即IDSW=2；在第7帧和第8帧中，目标A和B的ID又发生了切换，即IDSW=2。因此，MOTA的计算公式为：
$$
MOTA = 1 - \frac{FN + FP + IDSW}{GT} = 1 - \frac{2 + 2 + 4}{20} = 0.6
$$
MOTA的值为0.6，表示跟踪器的性能一般，还有改进的空间。



* **MOTP(多目标跟踪精确度)**

MOTP是多目标跟踪的精确度，用于衡量目标位置确定的精确程度。它的计算公式是：
$$
MOTP=\frac {\sum_ {t,i}d_ {t,i}} {\sum_tc_t}
$$
其中，t表示第t帧， c_t 表示第t帧中预测轨迹和GT轨迹成功匹配上的数目， d_ {t,i} 表示t帧中第i个匹配对的距离。这个距离可以用IOU或欧式距离来度量，IOU大于某阈值或欧氏距离小于某阈值视为匹配上了。

为了说明MOTP的计算过程，我假设有一个10帧的视频序列，其中有两个跟踪目标A和B，以及两个预测轨迹1和2。我用IOU作为距离度量，阈值设为0.5。下表是每一帧中的匹配情况，以及对应的IOU值：

| 帧   | 匹配对 | IOU  |
| ---- | ------ | ---- |
| 1    | (A,1)  | 0.8  |
| 1    | (B,2)  | 0.7  |
| 2    | (A,1)  | 0.9  |
| 2    | (B,2)  | 0.6  |
| 3    | (A,1)  | 0.7  |
| 3    | (B,2)  | 0.8  |
| 4    | (A,1)  | 0.6  |
| 4    | (B,2)  | 0.9  |
| 5    | (A,1)  | 0.5  |
| 5    | (B,2)  | 0.7  |
| 6    | (A,1)  | 0.4  |
| 6    | (B,2)  | 0.8  |
| 7    | (A,2)  | 0.6  |
| 7    | (B,1)  | 0.7  |
| 8    | (A,2)  | 0.8  |
| 8    | (B,1)  | 0.9  |
| 9    | (A,2)  | 0.7  |
| 9    | (B,1)  | 0.6  |
| 10   | (A,2)  | 0.9  |
| 10   | (B,1)  | 0.5  |

根据公式，我们可以计算出MOTP的值为：
$$
MOTP=\frac {0.8+0.7+0.9+0.6+0.7+0.8+0.6+0.9+0.5+0.7+0.4+0.8+0.6+0.7+0.8+0.9+0.7+0.6+0.9+0.5} {20}=0.71
$$
这个值表示预测轨迹和GT轨迹的平均IOU为0.71，越接近1表示越精确。



* 参考链接

[MOT和MTMC指标总结及详细计算方法_mot count ids-CSDN博客](https://blog.csdn.net/qq_34919792/article/details/107067992?spm=1001.2014.3001.5506) 

[【多目标跟踪任务——评价指标】_mota值多大-CSDN博客](https://blog.csdn.net/selami/article/details/124053187?ops_request_misc=&request_id=&biz_id=102&utm_term=MOTA%E8%AE%A1%E7%AE%97&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-1-124053187.142^v96^pc_search_result_base5&spm=1018.2226.3001.4187) 

[MOT多目标跟踪评价指标及计算代码（持续更新） - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/405983694) 

[MOT多目标跟踪评价指标代码py-motmetrics-CSDN博客](https://blog.csdn.net/sleepinghm/article/details/119538354) 



#### UKF算法

```shell
1. 求sigma point->(nx(2n+1))
2. 将得到的sigma point乘相应的状态转移矩阵生成一步预测值->(nx(2n+1))
3. 乘相应的权重得到均值，协方差->(nx1)
4. 1）根据状态一步预测值,代入观测方程再求观测预测值(nx(2n+1)) ->nx(2n+1))
   2）第二种方法是利用均值和协方差再产生新的sigmapoint去带入观测方程
5. 乘以相应的权重得到观测均值，状态预测协方差(nx1)
7. 利用最终预测值进行卡尔曼更新
```

* 参考链接：
* [无迹卡尔曼滤波UKF的理解与应用（附Matlab实例） - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/482392082) 
* [手撕自动驾驶算法——无迹卡尔曼滤波（UKF）-CSDN博客](https://blog.csdn.net/weixin_42905141/article/details/99710297?ops_request_misc=&request_id=&biz_id=102&utm_term=UKF&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-1-99710297.142) 



#### IMM算法

核心思想：一个模型对应一个卡尔曼滤波器，多个卡尔曼滤波器并行运行，每个滤波器的滤波值和滤波器的概率进行加权，得到最终输出。

> 1. 假设IMM算法中使用了两个滤波器，滤波器1的概率为0.8，滤波器2的概率为0.2，马尔科夫转移矩阵为：
>
> $$
> \mu_1=0.8, \mu_2=0.2\\
> 
> P = \left[
> \matrix{
>   0.98 & 0.02\\
>   0.02 & 0.98\\
> }
> \right]
> $$
>
> 2. 滤波器1经过状态转移之后由两部分组成，滤波器1的预测概率为：
>
> $$
> P_{00}\cdot\mu_1 + P_{10}\cdot\mu_2=0.98*0.8+0.02*0.2=0.788
> $$
>
> 3. 模型的混合概率指的是模型i到模型j预测概率过程中，模型i贡献了多少权重
>
>    
>
> 4. 因此对应上面，模型0到0的混合概率为：
>
> $$
> \lambda_{00}=(P_{00}\cdot\mu_1)/0.784=0.99492
> $$
>
> 5. 模型1到0的混合概率为：
>
>
> $$
> \lambda_{10}=(P_{10}\cdot\mu_2)/0.784=0.005076
> $$
>

然后将得到的混合概率与不同卡尔曼滤波的预测值相乘计算均值和协方差

参考链接：

* [卡尔曼滤波好文分享以及重点内容提取第四弹——IMM交互多模型 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/621375776) 



#### 概率论

* 贝叶斯估计

[【精选】Traditional Multiple Objects Tracking (updating)-CSDN博客](https://blog.csdn.net/qq_21933647/article/details/107433035?ops_request_misc=&request_id=&biz_id=102&utm_term=PDA%E7%AE%97%E6%B3%95&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduweb~default-1-107433035.142^v96^pc_search_result_base5&spm=1018.2226.3001.4187) 

* 张宇数学基础讲



#### 自动驾驶数据融合方法

* https://zhuanlan.zhihu.com/p/470588787 
* Multi-modal Sensor Fusion for Auto Driving Perception: A Survey 
* https://zhuanlan.zhihu.com/p/446284775



#### CTRV模型

* [过程噪声是指在状态转换模型中，由于物理过程的不确定性或者模型的简化而引入的随机扰动。](https://zhuanlan.zhihu.com/p/389589611)[1](https://zhuanlan.zhihu.com/p/389589611)[ 过程噪声通常假设为零均值的高斯白噪声，其协方差矩阵为Q。](https://www.cnblogs.com/jiangxinyu1/p/12448915.html) 

```shell
在CTRV模型中，假设物体的速度 (v) 和旋转速度 (ω) 为常数，而忽略了加速度的变化。因此，要把加速度项的影响放到过程噪声中。3 CTRV模型中的过程噪声可以分为两个部分：
1) 纵向加速度噪声 (va)，表示在纵向速度上的不确定性，因此可以定义为正态分布的白噪声，以0为均值，以σa^2为方差。
2) 横摆角加速度噪声 (αa)，表示在横摆角速度上的不确定性，因此可以定义为正态分布的白噪声，以0为均值，以σα^2为方差。
```

![1702091979138](C:\Users\GMM\AppData\Local\Temp\1702091979138.png)

![1702091995353](C:\Users\GMM\AppData\Local\Temp\1702091995353.png)

* [CRTV&UKF - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/561618286) 

![1702091785720](C:\Users\GMM\AppData\Local\Temp\1702091785720.png)



#### 数据关联算法

> * KNN算法：计算简单，缺点是多回波环境下离目标预测位置最近的候选回波不一定是目标的真实回波，适用于稀疏回波环境中跟踪非机动目标回波。
> * PDA算法：考虑了落入相关波门内的所有候选回波，并且根据不同的相关情况计算出各回波来自目标的概率，利用这些概率值；
>
> ![1700633544135](C:\Users\GMM\AppData\Local\Temp\1700633544135.png)

- 参考链接：
- [概率数据关联（PDA）算法解析 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/176851546) 
- [【精选】手撕自动驾驶算法——多目标跟踪：数据关联_数据关联实现多目标跟踪_令狐少侠、的博客-CSDN博客](https://blog.csdn.net/weixin_42905141/article/details/123754041?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522170062003116800222819795%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=170062003116800222819795&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~baidu_landing_v2~default-6-123754041-null-null.142^v96^pc_search_result_base5&utm_term=PDA%E7%AE%97%E6%B3%95&spm=1018.2226.3001.4187) 



#### 心得

* 会不会取决于想不想学

## 代码

#### 链表代码

```c++
#include <iostream>
#include <string.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <algorithm>
using namespace std;

struct ListNode{
	ListNode* next;
	int val;
	ListNode():val(0), next(nullptr){}
	ListNode(int x):val(x), next(nullptr){}
};

class MyList{
public:
	MyList(){
		_head = new ListNode(0);
		_size = 0;
	}

	//头插法
	void addAtHead(int val){
		ListNode* newNode = new ListNode(val);
		newNode->next = _head->next;
		_head->next = newNode;
		_size++;
	}

	void addAtTail(int val){
		ListNode* newNode = new ListNode(val);
		ListNode* cur = _head;
		while(cur->next != nullptr){
			cur = cur->next;
		}
		cur->next = newNode;
		_size++;
	}

	ListNode* getHead() const{
		return _head;
	}

	//反转整个链表
	ListNode* reverseList(ListNode* node){
		ListNode* pre = nullptr;
		ListNode* cur = node;
		while(cur){
			ListNode* tmp = cur->next;
			cur->next = pre;

			pre = cur;
			cur = tmp;
		}
		return pre;
	}

	//打印链表
	void printList(ListNode* node){
		ListNode* cur = node;
		while(cur != nullptr){
			if(cur->next == nullptr){
				cout << cur->val << endl;
			}else{
				cout << cur->val << "->";
			}
			cur = cur->next;
		}
	}

	//两两反转
	ListNode* swapPairs(ListNode* head){
		ListNode* cur = head;
		while(cur->next && cur->next->next){
			ListNode* tmp1 = cur->next;
			ListNode* tmp2 = cur->next->next->next;

			cur->next = cur->next->next;
			cur->next->next = tmp1;
			cur->next->next->next = tmp2;

			cur = cur->next->next;
		}
		return head->next;
	}

	//删除链表倒数第n个节点
	ListNode* removeNthFromEnd(ListNode* head, int n){
		ListNode* fast = head;
		ListNode* slow = head;
		while(n-- && fast){
			fast = fast->next;
		}
		while(slow->next && fast->next){
			fast = fast->next;
			slow = slow->next;
		}
		ListNode* tmp = slow->next;
		slow->next = slow->next->next;
		delete tmp;
		return head->next;
	}

	//环型链表
	ListNode* detectCycle(ListNode* head){
		ListNode* fast = head;
		ListNode* slow = head;
		while(fast && fast->next){
			fast = fast->next->next;
			slow = slow->next;
			if(fast->val == slow->val){
				ListNode* cur1 = fast;
				ListNode* cur2 = head;
				while(cur1->val != cur2->val){
					cur1 = cur1->next;
					cur2 = cur2->next;
				}
				return cur1;
			}	
		}
		return NULL;
	}

private:
	ListNode* _head;
	int _size;
};


int main(){
	MyList *A = new MyList();

	A->addAtHead(1);
	A->addAtTail(4);
	A->addAtTail(5);
	A->addAtTail(8);
	A->addAtTail(7);

	cout << "Before List: " << endl;
	A->printList(A->getHead()->next);

	// cout << "After reverse: " << endl;
	// ListNode* reverseHead = A->reverseList(A->getHead()->next);
	// A->printList(reverseHead);

	ListNode* swapList = A->swapPairs(A->getHead());
	A->printList(swapList);

	// ListNode* removeList = A->removeNthFromEnd(A->getHead(), 2);
	// A->printList(removeList);

	delete A;
	return 0;
}
```
