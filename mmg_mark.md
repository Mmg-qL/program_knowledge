

## 一、Linux内核

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

## 二、C++和C语言

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





## 三、计算机网络

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

## 四、调试相关

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
   >         		* label
   >         		* pose
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



## 五、ROS

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

   

#### ROS通信方式

1. 话题通信
2. 服务通信
3. 参数服务器



#### 激光雷达目标检测

1. 体素分割->重新拼接->张量投影到二维平面->作一个类似于Poinpillars的2D伪图像
2. 降采样->升维
3. 类似SSD的检测头设置 ->设置IoU阈值



#### 感知融合

3D检测框信息投影到2D平面，计算IoU，如果超过一定阈值则进行匹配，这个匹配的检测框被赋予二维相机检测的属性



#### 跟踪算法

>UKF：本质是对非线性卡尔曼滤波的线性化，利用SigmaPoint去拟合；比如我用的是5维的状态向量分别是x,y,v,w,w'；通过SigmaPoint去拟合非线性，就得到11维度的SigmaPoint；再去求一步预测，进而求加权平均；再去求先验估计协方差和新息协方差；后验估计以及逐步递推；
>
>IMM：交互式多模型卡尔曼滤波；使用一种系统动态模型的卡尔曼滤波器对于多种运动状态的跟踪预测不匹配；比如我有两个状态对应两个滤波器，模型之间有一个2x2的转移矩阵；分别去求他们的权重，不断更新；再去加权求卡尔曼滤波；
>
>PDA算法：在实际的卡尔曼滤波中有预测的轨迹和我量测的轨迹，交杂在一起，这个算法的主要目的是判定轨迹关联，除了这个算法还有KNN算法，基于马氏距离的判定，JPDA, CJPDA。为每个有效量测计算一个概率，并结合新息计算出最优量测。



## 六、算法相关

#### UKF算法

1. 求sigma point

2. 将得到的sigma point乘相应的状态转移矩阵

3. 乘相应的权重得到均值，协方差

4. 根据预测值再求sigma point

5. 带入观测方程

6. 乘以相应的权重得到均值，协方差

7. 利用最终预测值进行卡尔曼更新

   

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



#### 数据关联算法

> * KNN算法：计算简单，缺点是多回波环境下离目标预测位置最近的候选回波不一定是目标的真实回波，适用于稀疏回波环境中跟踪非机动目标回波。
>
> * PDA算法：考虑了落入相关波门内的所有候选回波，并且根据不同的相关情况计算出各回波来自目标的概率，利用这些概率值；





## 七、代码

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



#### 华测导航

> * 为什么投华测？离家近，师兄在里面，实验室合作项目
> * 为什么不去经纬恒润？家在上海，南京可以转正我拒绝了，之前一直在等华测官网开
