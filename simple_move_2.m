	% 各连杆参数（虚拟）  
	l1= 0.08;  
	l2= 0.2;  
	l3= 0.2;   
	% 基本偏置参数  
	thetaVal = zeros(4,1);    
	% 定义各个连杆以及关节类型，默认为转动关节  
	%           theta    d        a        alpha  
	%                连杆偏距d 连杆长度  关节偏角alpha  
	L1=Link([      0     0        0          0],    'modified'); % [四个DH参数], options  
	L2=Link([      0     0        l1     -pi/2],    'modified');  
	L3=Link([      0     0        l2         0],    'modified');  
	L4=Link([      0    ,0        l3         0],    'modified');  
	% 将连杆组成机械臂  
	robot=SerialLink([L1,L2,L3,L4]);   
	robot.name='singleLeg';  
	robot.offset=thetaVal;
	% 基本演示
	robot.display(); 
	view(3); 
	% 解决robot.teach()和plot的索引超出报错
	robot.teach();
	% 五项式轨迹规划动态
	% 设定位姿为（0.4,0.1,-0.1 -- 0.4,0.1,0.1）
	%根据起始点位姿，得到起始点关节角
	q1=[-14.036*pi/180 46.76*pi/180 -60*pi/180 0];
	%根据终止点位姿，得到终止点关节角
	q2=[-14.036*pi/180 13.24*pi/180 -60*pi/180 0];
	%五次多项式轨迹，得到关节角度，角速度，角加速度，50为采样点个数
	[q ,qd, qdd]=jtraj(q1,q2,50); 
	grid on
	%根据插值，得到末端执行器位姿
	T=robot.fkine(q);
	nT=T.T; 
	plot3(squeeze(nT(1,4,:)),squeeze(nT(2,4,:)),squeeze(nT(3,4,:)));
robot.plot(q,'workspace',[-40 40 -40 40 -40 40],'delay',0.001,'fps',120,'trail','b','view',[30,50]);
% 注：在实际的简单操作中可以直接使用默认options，故只输入关节角度即可
	%输出末端轨迹
	hold on
	%动画演示
	robot.plot(q);

