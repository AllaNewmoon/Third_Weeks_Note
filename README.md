# Second_Weeks_Note
 # 机器人学动力学
 ## 旋转矩阵
 1.**投影法求旋转矩阵**：设A为世界坐标系，B为机器人坐标系，**^A^R~B~ = [X~B~ * X~A~, Y~B~ * Y~A~, Z~B~ * Z~A~]^T^**，其余同理。旋转矩阵Q^-1^ = Q^T^。

 2.**旋转矩阵转换向量坐标**：^A^P = ^A^R~B~ * ^B^P。

3.**旋转矩阵描述物体转动状态**：绕z轴旋转R~Z~(θ) = [cθ -sθ 0, sθ cθ 0, 0 0 1]^T^ = [R 0, 0 1]^T^，绕x轴为[1 0, 0 R]^T^

## 旋转矩阵与转角
1.Fixed Angles(eg:X-Y-Z):**R~XYZ~ = R~Z~R~Y~R~X~**, v^'^ = R~1~R~2~R~3~v, 利用Atan2由R求出角度。

2.Euler Angles(eg:Z-Y-X)：^A^R~B~ = R~Z~R~Y~R~X~，P~A~ = R * P~B~.(Euler反转效果和fixed angle正转效果相同)

3.齐次矩阵^A^T~B~ = [^A^R~B~ ^A^P~B~, 0 1](齐次矩阵^A^T~B~*^B^T~A~ = I)

（相关函数：rotx，roty，rotz，eul2r，eul2tr，rpy2r，rpy2tr，tr2eul，tr2rpy，transl）

## 运动学
### 顺向运动学(FK)：
Link0：地杆，不动。Link1，Link2依次类推。

Link Length指两转轴之间的直线距离a，Link Twist指两转轴之间夹角alpha。

Link Offset指连杆偏距d，是两个相邻连杆在公共轴上的距离。Joint angle指连杆之间的转角theta。(对于地杆，记a=0，d=0)

从Axis i-1的轴Zi-1到Axis i的轴Zi，需要变换alpha，a，theta，d四次。^i-1^P = ^i-1^T~R~^R^T~Q~^Q^T~P~^P^T~i~^i^P = ^i^T~i-1~P~i~(Standard)
![](https://github.com/AllaNewmoon/123/blob/main/standard.png?raw=true)

**DH表示法**：linki后面是jointi，theta变为X~i-1~和X~i~之间的夹角，d变为X~i-1~和X~i~之间的距离，其余同理。^i-1^T~i~ = T~zi-1~T~ZR~T~XQ~T~XP~(Craig)

(Craig方法中^3^T~0~*T~X3~等于standard方法中^3^T~0~)
![](https://github.com/AllaNewmoon/123/blob/main/craig.png?raw=true)

(**^i-1^T~i~的一般式子：)

### 逆向运动学(IK)：
给予^w^P求得θ的过程，顺向运动学的反过程。

Reachable Workspace：手臂可以用一种以上姿态到达的位置

Dexterous Workspace：手臂可以用任何姿态到达的位置

Subspace：手臂在定义头尾的T所能达到的变动范围

**求多重解**：

1.解析法：

几何法：将空间几何切割成平面几何。利用正弦和余弦定理求解角度。

代数法：建立方程

（eg：C~θ~ = C~123~, S~θ~ = S~123~, x = l~1~C~1~+l~2~C~12~, y = L~1~S~1~+L~2~S~12~。x的平方加y的平方可解得C~2~，求逆得θ~2~，再带回x，y式子可解得θ~1~，）
![](https://github.com/AllaNewmoon/123/blob/main/ikproblem.png?raw=true)

2.Pieper's Solution（以六个自由度，前三个joint移动，后三个轴交于一点转动为例）；（移动部分）采用^i-1^T~i~的通式进行递推，最后得到目标x，y，z与θ~1~，θ~2~，θ~3~的函数式的关系。具体如下：
![](https://github.com/AllaNewmoon/123/blob/main/piepersolution.png?raw=true)
![](https://github.com/AllaNewmoon/123/blob/main/piepersolution2.png?raw=true)
![](https://github.com/AllaNewmoon/123/blob/main/piepersolution3.png?raw=true)

（转动部分）θ1，θ2，θ3已知，则有^3^R~6~ = inv(^0^R~3~)*^0^R~6~。可以用Z-Y-Z Euler Angle求解。设DH表示法下有θ4，θ5，θ6，则在euler angle下为θ4^'^ = θ4+pi，θ5^'^ = θ5，θ6^'^ = θ6+pi。
![](https://github.com/AllaNewmoon/123/blob/main/euler.png?raw=true)


## 轨迹规划
1.Joint Space下轨迹规划：将^G^T~T~以六个参数（旋转+转动）的形式表达，再将手臂末端点状态转换到joint状态(通过ik，^G^X~T~ -> Θ~i~)，对所有joint规划smmoth trajectories，将joint状态转换到手臂末端状态。

2.Cartesian Space下轨迹规划：先做trajectory planning，再用ik，检查joint状态在joint space下的可行性。

**轨迹规划方法**：

1.Cubic Polynomials(三次多项式)：不同轨迹区域[ti ti+1]以不同参数的函数来规划，需定义各函数的边界条件(θ(ti), θ(ti+1), θ^'^(ti), θ^'^(ti+1))，有四个条件，得四个方程，用矩阵逆运算求得三次多项式的参数a0, a1, a2, a3。
![](https://github.com/AllaNewmoon/123/blob/main/cubicpolynomials.png?raw=true)
![](https://github.com/AllaNewmoon/123/blob/main/cubicpolynomials2.png?raw=true)

2.多段Cubic Polynomials，速度条件：如果θi在ti前后变号，选择θ^'^i = 0，否则选择平均。

（一般情况下N段曲线有4N个方程，其中起始点，终点边界条件可得2N个，中间点的速度，加速度连续可得2（N-1）个，最后定义头尾的速度，加速度可得2个方程）

对多段矩阵，则有```Θ~N~ = T~N*N~A~N~```，在Cartesian Space下若将X，Y，θ都带入，则有```Θ~N*3~ = T~N*N~A~N*3~```，在joint space下则用ik求出joint angles，再带入上式。

（如果位置，速度，加速度都要规划，采用五次多项式）

**Linear Function with Parabolic Blends**：轨迹中包含多个直线段。(在matlab中采用mtraj(@lspb/@tpoly, 起始位姿 终点位姿 时间步)进行三角度姿态插值，用trinterp(T0, T1, 时间步)进行位姿插值)

eg：利用二次曲线连接直线（设出二次式，联立方程，将未知的时间解出来）：![](https://github.com/AllaNewmoon/123/blob/main/blends.png?raw=true)
![](https://github.com/AllaNewmoon/123/blob/main/blends2.png?raw=true)

对于多段二次线段，可以设定加速度，从而解出时间，也可以设定时间解出加速度。



## SerialLink基础
1.模型展示：R.plot(Q, options)，q是1 * n矩阵，n是自由度。option包含"view(az，el)"，az是和-y的夹角，el是和xy平面的夹角。'workspace'指工作空间大小，例如[800 -800 800 -800 0 1000]，指x轴范围-800到800，y轴范围-800到800，z轴范围0到1000。

2.DH表示法建立模型：L(i) = Link('revolute','d','a','alpha','offset')，revolute表示旋转，offset表示θ。用five_dof = SerialLink(L,'name','5-dof')串联link，用five_dof.base = transl()指定基座。

也可以采用Link([theta d a alpha offset],'standard')的方式指定。view(3)以三维角度看

eg：
```
>> %        theta  d        a        alpha    offset
L1 = Link([0     0           0        -pi/2    0],'standard');
L2 = Link([0     0.19        0.650    0        0],'standard');
L3 = Link([0     0           0        pi/2     0],'standard');
L4 = Link([0     0.600       0        -pi/2    0],'standard');
L5 = Link([0     0           0        pi/2     0],'standard');
L6 = Link([0     0           0        0        0],'standard');
robot=SerialLink([L1 L2 L3 L4 L5 L6],'name','PUMA 762');
theta=[-pi/2 -pi/2 pi/2 0 0 0];
view(3)
robot.plot(theta);
teach(robot);
```

3.plot函数再探：以四轴机械臂为例
```
clear ; clc; close all;
% 机器人各连杆DH参数
d1 = 0;
d2 = 86;
d3 = -92;
% 由于关节4为移动关节，故d4为变量，theta4为常量
theta4 = 0;

a1 = 400;
a2 = 250;
a3 = 0;
a4 = 0;

alpha1 = 0 / 180 * pi;
alpha2 = 0 / 180 * pi;
alpha3 = 180 / 180 * pi;
alpha4 = 0 / 180 * pi;
% 定义各个连杆，默认为转动关节
%           theta      d        a        alpha 
L(1)=Link([  0         d1      a1      alpha1]); L(1).qlim=[-pi,pi];
L(2)=Link([  0         d2      a2      alpha2]); L(2).qlim=[-pi,pi]; L(2).offset=pi/2;
L(3)=Link([  0         d3      a3      alpha3]); L(3).qlim=[-pi,pi];
% 移动关节需要特别指定关节类型--jointtype
L(4)=Link([theta4       0      a4      alpha4]); L(4).qlim=[0,180]; L(4).jointtype='P';
% 把上述连杆“串起来”
Scara=SerialLink(L,'name','Scara');
% 定义机器人基坐标和工具坐标的变换
Scara.base = transl(0 ,0 ,305);
Scara.tool = transl(0 ,0 ,100);
view(3)
Scara.teach();   
```
qlim函数指定活动范围，需要注意移动关节需特别指定jointtype='P'。

如果指定joint=[pi/6 0 pi/3 150]，scara.plot(joint)则绘制关节变量为joint时的机器人状态。如果joint是数组，则绘制机器人运动动画。
```
joint(: , 1) = linspace(pi/6,pi/2,100);
joint(: , 2) = linspace(0,pi/4,100);
joint(: , 3) = linspace(pi/3,pi/2,100);
joint(: , 4) = linspace(0,160,100);
Scara.plot(joint ,'jointdiam',1,'fps',100,'trail','r-')
```
jointdiam设置关节圆柱体直径大小，fps设置帧率，trail绘制末端轨迹。以上代码用linspace创建变化，第一个关节放在第一列，依次类推。
```
filename = 'demo.gif';
for i = 1:length(joint)
    pause(0.01)
    Scara.plot(joint(i,:));
    f = getframe(gcf);  
    imind = frame2im(f);
    [imind,cm] = rgb2ind(imind,256);
    if i == 1
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',0.1);
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.1);
    end
end
```
joint是4*100的矩阵，利用for循环遍历每一行，即为每个时刻的关节位置。getframe捕获图窗窗口内部区域，

3.**matlab正逆运动学求解**：利用fkine求解正运动学，输入关节角度，输出SE3齐次变换矩阵。利用ikine解逆运动学，输入齐次变换矩阵，求解关节向量。ikine的常用可选项：'mask',[1 1 1 0 0 1]，分别指x，y，z移动自由度和转动自由度。

使用实例：
```
T0=robot.fkine(init);
	TF=robot.fkine(targ);
	% 得到机器人在变换过程中每一步（step）的齐次变换矩阵
	step = 200；
	TC=ctraj(T0,TF,step);
	% 逆运动学计算
	qq=robot.ikine(TC,'mask',[1 1 1 0 0 0]);
```

4.jtraj函数轨迹规划：jtraj函数输入起始点位姿(joint angle)，终点位姿，输出各关节角度，角速度，角加速度。

# 链表
1.结构定义：链表是一串节点，每个节点存储两个信息，第一个信息是数据，第二个信息是下一个节点的地址。**head**变量指向首地址。程序内部的信息控制内存内部的信息。

2.链表插入元素：node节点指向待插入元素地址，用一个指针指向前一个元素，让新的元素指向后一个元素，再让前一个元素指向新元素。

*如果先让前一个元素指向新元素，则后面的元素会**内存泄漏***

3.**无头链表**：头部不存储信息，相当于用指针指向链表。

**有头链表**：头部是一个节点，有存储数据的区域。
## 链表实现
```
typedef struct node{
    int data;//数据信息
    struct* next;//指针信息
} Node;
//初始化新链表(给一个值，返回节点)
Node* getNewNode(int val){
    Node *p = (Node *)malloc(sizeof(Node));
    p->data = val;
    p->next = NULL;
    return p;
}
//销毁链表
void clear(Node *head){
    if(head==NULL)
        return;
    for(Node *p = head,q;p;p = p->next){
        q = p->next;//用node q存储下一个节点，防止内存泄漏
        free(p);
    }
}
//插入操作(无头链表插入后返回新链表首地址)
Node *insert(Node *head,int pos,int val){
    if(pos == 0){
        Node *p = getNewNode(val);
        p->next = head;
        return p;
    }
    Node *p = head;//让p指针指向前一个元素
    for(int i = 1;i<pos;i++){
        p = p->next;
    }
    Node *node = getNewNode(val);
    node->next = p->next;
    p->next = node;
    return head;
}
//输出链表
void output_linklist(Node *head){
    int len = 0, n = 0;
    for(Node *p = head;p;p = p-next)
        n += 1;//统计一共有几个节点
    for(int i = 0; i<n; i++){
        printf("%3d", i);
        printf("  ");
    }
    printf("\n");
    for(Node *p = head;p;p = p-next){
        printf("%3d", p->data);
        printf("->");
    }
    printf("\n\n\n");
    return ;
}
//查找链表里的元素
int find(Node *head,int val){
    Node *p = head;
    int n = 0;
    while(p){
        if(p->data == val){
            output_linklist(head);
            int len = n*(3+2)+2;
            for(int i = o; i<len; i++){
                printf(" ");
            }
            printf("^\n");
            for(int i = o; i<len; i++){
                printf(" ");
            }
            printf("|")
        }
        n += 1;
        p = p->next;
    }
}
```
**有头链表的插入操作**：
```
Node *insert(Node *head, int pos, int val){
    Node new_head, *p = &new_head;
    new_head.next = head;//创建虚拟头
    for(int i = 0; i < pos; i++)
        p = p->next;
    node->next = p->next;
    p->next = node;
    return new_head.next;
}
```  
**循环链表**：1.单向循环链表：最后一个节点指向第一个节点，把head看作整个单向循环链表的尾节点。

2.双向链表：结构定义中多了一个pre指针指向前一个元素

**链表的其它操作**：1.链表头插法：将元素不断插到new_head后面，可以实现反转链表。

2.判断环形链表：用两个指针，其中一个快另一个慢，若两个指针能相遇，则有环。可解决快乐数等问题。（双指针思想）
![](https://github.com/AllaNewmoon/123/blob/main/%E7%8E%AF%E5%BD%A2%E9%93%BE%E8%A1%A8%E5%BF%AB%E6%85%A2%E6%8C%87%E9%92%88.png?raw=true)

3.旋转链表：双指针思想运用，p指向尾元素，q指向待旋转链表前一个元素，得到待插入链表。再令q=p，让q遍历待旋转链表，令q->next指向首元素即可，此时p为首元素，return p
```
ListNode* rotateRight(ListNode* head, int k) {  
        if(head == NULL){
            return head;
        }
        int n = getlength(head);
        k %= n;
        if(k == 0){
            return head;
        }
        ListNode *p = head, *q = head;
        for(int i = 0;i <= k;i++){
            p = p->next;
        }
        while(p){
            p = p->next;
            q = q->next;
        }
        p = q->next;//p指向待转移链表的第一个元素
        q->next = NULL;//不转移链表的尾部指向空指针
        q = p;
        while(q->next != NULL){
            q = q->next;
        }
        q->next = head;
        return p;
    }
```
    
3.删除倒数第N个节点：采用双指针等距移动法加上虚拟头节点（将无头链表变为有头链表）
![](https://github.com/AllaNewmoon/123/blob/main/%E5%8F%8C%E6%8C%87%E9%92%88%E7%AD%89%E8%B7%9D%E7%A7%BB%E5%8A%A8.png?raw=true)
    
```
ListNode* removeNthFromEnd(ListNode* head, int n){
    ListNode new_head, *p = &new_head, *q = p;
    new_head.next = head;//创建有头链表
    for(int i = 0; i<=n; i++)
        q = q->next;
    while(q)
        p = p->next,q = q->next;
    p->next = p->next->next;//跳过待删除节点
    return new_head.next;
}
```
# 栈和队列
## 队列
1.结构定义：队列保持从队首出元素，尾部加入元素。指向队首和队尾的指针左闭右开。类比现实生活的排队买票

循环队列：对队列的存储空间可以循环使用，tail超出空间时指向0号元素。

**2.队列实现**：

用顺序表实现队列：
```
typedef struct Queue{
    vector *data;
    int size, head, tail//空间大小，头指针。尾指针，元素个数
} Queue;
//初始化操作
Queue *initQueue(int n){
    Queue *q = (Queue *)malloc(sizeof(Queue))
    q->data = initVector(n);
    q->size = n;
    q->head = q->tail = q->count;
}
//压入操作
int push(Queue *q, int val){
    if(q->count == q->size)
        return 0;
    insertVector(q->data, q->tail, val);
    q->tail += 1;
    if(q->tail == q->size)
        q->tail = 0;//已经到队尾，则重新指向队首
    return 1;
}
//弹出操作
int pop(Queue *q){
    if(empty(q))
        return 0;
    q->head += 1;
    q->count -= 1;
    return 1;
}
//查看队首元素
int front(Queue *q){
    return vectorSeek(q->data, q->head);
}
//判空操作
int empty(Queue *q){
    return q->count == 0;
}
//清除队列
void clearQueue(Queue *q){
    if(q == NULL)
       return ;
    clearVector(q->data);
    free(q);
    return;
}
```
改成链表实现时不需要循环队列和首尾指针。
## 栈
1.结构定义：先进后出，只能一头进，一头出，想把先放进去的元素取出来必须先把后放进去的元素取出来。入栈和出栈顺序可能不一样。

2.栈实现：
```
typedef struct Stack{
    int *data;
    int size, top;//top记录栈顶的下标
} Stack;
//初始化
Stack *initStack(int n){
    Stack *s = (Stack *)malloc(sizeof(Stack));
    s->data = (int *)malloc(sizeof(int)*n);
    s->size = n;
    s->top = -1;
    return s;
}
//清除栈
void clearStack(Stack *s){
    if(s == NULL)
        return;
    free(s->data);
    free(s);
    return;
}
//栈判空
int empty(Stack *s){
    return s->top == -1;
}
//查看栈顶
int top(Stack *s){
    if(empty(s))
        return 0;
    return s->data[s->top];
}
//入栈
int push(Stack *s, int val){
    if(s->top + 1 == s->size)
        return 0;
    s->top += 1;
    s->data[s->top] = val;
    return 1;
}
//出栈
int pop(Stack *s){
    if(empty(s))
        return 0;
    s->top -= 1;
    return 1;
}
```
**栈的其它操作**：

1.辅助栈法，利用栈先出后入的特点解决问题。eg1：有效的括号问题，遇到左括号则进栈，遇到匹配的右括号则让左括号出栈，最后判断栈是否为空。同时可以用数组，列表代替栈。

eg2（HZOJ595）：
```
int main(){
    int flag = 0,n;
    cin >> n;
    vector<string> ops(n), s;//用数组s模拟栈
    string target;
    for(int i = 0;i < n;i++)
        cin >> ops[i];
    cin >> target;
    for(int i = 0;i < n;i++){
        if(target == ops[i]){
            s.push_back(ops[i])//将数组尾端当成栈顶
            flag = 1;
            break;
        }
        if(ops[i] == "return")
            s.pop_back();//popback模拟出栈操作
        else s.pushback(ops[i])
    }
    if(flag){
        for(int i = 0;i<s.size();i++){
            if(i)
        }
    }
}
```
2.C++自带的栈stack操作：s.pop(), s.empty(), s.push().

3.验证栈序列:给定入栈顺序的元素,判断能不能按照给定的顺序进行出栈.(eg:给定[1 2 3 4 5], popped = [4 5 3 2 1]可行)
```

# 机器人导航
## 1.反应式导航
在没有任何地图信息，没有路径规划的情况下完成任务。

**eg1.Braitenberg车**: 建立感知函数，利用传感器对其参数化，寻求一个标量场的最大值。在本例中被感知势场由平方反比函数定义，通过感知函数得到s(x,y)，当s最大时小车速度为0。sl_braitenberg模型控制小车向最大值点移动。
![](https://github.com/AllaNewmoon/123/blob/main/QQ%E5%9B%BE%E7%89%8720230331214550.png?raw=true)

**eg2.简单自动机**：能避开不可达区域和障碍物，具有记忆功能。采用Bug2算法，先加载地图，bug.goal指定目标，bug.query([20;10])指定初始位置，运行仿真。

（bug2算法采取沿一条直线向目标点移动，遇到障碍物就逆时针绕行，直到回到最初直线）

***创建地图**：可以创建全为0的矩阵map = zeros(100, 100)，然后令map(40:50,20:80) = 1，也可以用map = makemap(100)。*

## 2.基于地图的路径规划

**1.距离变换**：将代表目标的非零元素矩阵经过距离变换可得到另一个大小相同的矩阵，其中每个元素的值是到最初非零目标元素的距离。

采取navigation类中的对象DXform将距离转换用于机器人导航。dx = DXform(map)创建对象，dx.plan(goal), dx.plot(p)，分别规划路径，显示出来。dx.path(start)显示过程动画。

（如果令p = dx.query(start)，则dx.plot(p)显示通过的点）

**2.D*算法**：寻找一条路径使运行的总成本最低。

使用工具箱实现D*规划：
```
ds = Dster(map);
c = ds.costmap();
ds.plan(goal);
q = ds.query(start);
ds.plot(q);
```
D*可以在任务中有效更改成本地图，采用modify_cost说明要做的修改。

eg：
```
for y=78:85
    for x=12:45//创建一片矩形区域
        ds.modify_cost([x,y], 2);//将通过矩形的代价设为2
    end
end
```

**3.沃罗诺伊路线图法**：

距离变换与D*算法存在规划阶段需要大量计算，改变目标就要重新计算的弊端。路线图建立了一个绕开障碍物的路网，只需计算一次就能去到任何目标位置。
```
free = 1 - map;//创建自由空间
free(1,:) = 0; free(100,:) = 0;
free(:,1) = 0; free(:,100) = 0;//将最外围设为非自由空间
skeleton = ithin(free);//利用“细化”形态学图像处理算法产生拓扑骨架
```
**4.概率路线图方法**：

解决骨架化方法计算量大的问题，给予概率在全局地图上稀疏采样，称为PRM法。

eg：
```
load map1              % load map
         goal = [50,30];        % goal point
         start = [20, 10];      % start point
         prm = PRM(map);        % create navigation object
         prm.plan()             % create roadmaps
        q = prm.query(start, goal)  % animate
        prm.plot(q)
```
注意plan时不输入goal，该方法通过找100个随机点，通过直线将该点与相邻点连接（不通过障碍），形成节点最小，开环的网络图。

**5.RRT**：

快速搜索随机数，步骤为：先保持一份机器人的位姿图，每个节点对应一个机器人的位姿。再选择一个随机节点rand，找出与其最近的节点near，计算指令使机器人能在固定时间段从near移动到rand，其中达到的点记为new，并加入图形中。
![](https://github.com/AllaNewmoon/123/blob/main/rrt.png?raw=true)
