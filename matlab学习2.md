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

**DH表示法**：linki后面是jointi，theta变为X~i-1~和X~i~之间的夹角，d变为X~i-1~和X~i~之间的距离，其余同理。^i-1^T~i~ = T~zi-1~T~ZR~T~XQ~T~XP~(Craig)

(Craig方法中^3^T~0~*T~X3~等于standard方法中^3^T~0~)

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

2.Pieper's Solution（以六个自由度，前三个joint移动，后三个轴交于一点转动为例）；（移动部分）采用^i-1^T~i~的通式进行递推，最后得到目标x，y，z与θ~1~，θ~2~，θ~3~的函数式的关系。具体如下：

（转动部分）θ1，θ2，θ3已知，则有^3^R~6~ = inv(^0^R~3~)*^0^R~6~。可以用Z-Y-Z Euler Angle求解。设DH表示法下有θ4，θ5，θ6，则在euler angle下为θ4^'^ = θ4+pi，θ5^'^ = θ5，θ6^'^ = θ6+pi。

## 轨迹规划
1.Joint Space下轨迹规划：将^G^T~T~以六个参数（旋转+转动）的形式表达，再将手臂末端点状态转换到joint状态(通过ik，^G^X~T~ -> Θ~i~)，对所有joint规划smmoth trajectories，将joint状态转换到手臂末端状态。

2.Cartesian Space下轨迹规划：先做trajectory planning，再用ik，检查joint状态在joint space下的可行性。

**轨迹规划方法**：

1.Cubic Polynomials(三次多项式)：不同轨迹区域[ti ti+1]以不同参数的函数来规划，需定义各函数的边界条件(θ(ti), θ(ti+1), θ^'^(ti), θ^'^(ti+1))，有四个条件，得四个方程，用矩阵逆运算求得三次多项式的参数a0, a1, a2, a3。

2.多段Cubic Polynomials，速度条件：如果θi在ti前后变号，选择θ^'^i = 0，否则选择平均。

（一般情况下N段曲线有4N个方程，其中起始点，终点边界条件可得2N个，中间点的速度，加速度连续可得2（N-1）个，最后定义头尾的速度，加速度可得2个方程）

对多段矩阵，则有```Θ~N~ = T~N*N~A~N~```，在Cartesian Space下若将X，Y，θ都带入，则有```Θ~N*3~ = T~N*N~A~N*3~```，在joint space下则用ik求出joint angles，再带入上式。

（如果位置，速度，加速度都要规划，采用五次多项式）

**Linear Function with Parabolic Blends**：轨迹中包含多个直线段。(在matlab中采用mtraj(@lspb/@tpoly, 起始位姿 终点位姿 时间步)进行三角度姿态插值，用trinterp(T0, T1, 时间步)进行位姿插值)

eg：利用二次曲线连接直线（设出二次式，联立方程，将未知的时间解出来）：

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
