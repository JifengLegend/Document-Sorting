​		





# S 函数

## 1. 代码讲解

​	模板函数（在命令行窗口中输入`edit sfuntmpl`即可自动生成）

​	模板函数如下：

```matlab
function [sys,x0,str,ts,simStateCompliance] = sfuntmpl(t,x,u,flag)
switch flag,
%初始化%
case 0，
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
case 1,
    sys=mdlDerivatives(t,x,u);%更新%
case 2,
    sys=mdlUpdate(t,x,u);%输出%
case 3,
    sys=mdlOutputs(t,x,u);%获取next var hit 的时间%  
case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);%终止  
case 9,
    sys=mdlTerminate(t,x,u);%意外flag  
otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
end
```




### 1. 第一行是这样的：

```matlab
function [sys,x0,str,ts,simStateCompliance] = sfuntmpl(t,x,u,flag)
```

这是简写版：

~~~matlab
[sys,x0,str,ts]=sfuntmpl(t,x,u,flag)
~~~

其中函数参数：

| 名称 | 说明             |
| :--: | ---------------- |
|  t   | 采样时间         |
|  x   | 状态变量         |
|  u   | 当前输入变量     |
| flag | 仿真时的状态标志 |

其中返回参数：

| 名称 | 说明                                              |
| :--: | ------------------------------------------------- |
| sys  | 根据flag状态的不同而不同                          |
|  x0  | 状态变量的初始值                                  |
| str  | 保留参数，设为`[]`即可                            |
|  ts  | 一个1*2的向量；`ts[1]`：采样周期  `ts[2]`：偏移量 |

其中 flag ：

|  值  | 说明              | 说明                                                         |
| :--: | ----------------- | ------------------------------------------------------------ |
|  0   | [SIZES,X0,STR,TS] | 初始化，在SYS中返回system size，在X0中返回初始状态，在STR中返回状态排序字符串，在TS中返回采样时间。 |
|  1   | DX                | 返回SYS中的连续状态导数。                                    |
|  2   | DS                | 更新离散状态 `SYS = X(n+1)`                                  |
|  3   | Y                 | 返回SYS中的输出。                                            |
|  4   | TNEXT             | 返回SYS中可变步长采样时间的下一次hit时间。                   |
|  5   |                   | 预留给未来（寻根）。                                         |
|  9   | []                | 终止，执行任何清理SYS=[]。                                   |

## 2. 具体结构

#### case 0

```matlab
case0,
[sys,x0,str,ts]=mdlInitializeSizes;
```

```matlab
%初始化函数 mdlInitializeSizes
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes


sizes = simsizes;	%用于设置模块参数的结构体,用simsizes来生成

sizes.NumContStates  = 0;	%模块连续状态变量的个数
sizes.NumDiscStates  = 0;	%模块离散状态变量的个数
sizes.NumOutputs     = 0;	%模块输出变量的个数
sizes.NumInputs      = 0;	%模块输入变量的个数
sizes.DirFeedthrough = 1;	%模块是否存在直接贯通（直接贯通我的理解是输入能 直接控制输出）
sizes.NumSampleTimes = 1;   %模块的采样时间个数,最少需要一个初始时间
sys = simsizes(sizes);	%设置完后赋给sys输出
x0  = [];
% str 基本取空矩阵
str = [];
% 初始化采样次数数组
ts  = [0 0];
simStateCompliance = 'UnknownSimState';
```

举个例子，考虑以下模型：

- $dx/dt=fc(t,x,u)$ 也可以用连续状态方程描述：$dx/dt=A*x+B*u$

- $x(k+1)=fd(t,x,u)$ 也可以用离散状态方程描述：$x(k+1)=H*x(k)+G*u(k)$

- $y=fo(t,x,u)$ 也可以用输出状态方程描述：$y=C*x+D*u$

设上述模型连续状态变量、离散状态变量、输入变量、输出变量均为1个，我们就只需改上面那一段代码为：（一般连续状态与离散状态不会一块用，我这儿是为了方便说明）

```matlab
sizes.NumContStates=1;
sizes.NumDiscStates=1;
sizes.NumOutputs=1;
sizes.NumInputs=1;
```

其他的可以不变。继续在`mdlInitializeSizes`函数中往下看：

```matlab
x0=[];	%状态变量设置为空，表示没有状态变量，以我们上面的假设，可改为x0=[0,0](离散和连续的状态变量我们都设它初值为0)
str=[];	%这个就不用说了，保留参数嘛，置[]就可以了，反正没什么用，可能7.0会给它一些意义
ts=[00]; %采样周期设为0表示是连续系统，如果是离散系统在下面的mdlGetTimeOfNextVarHit函数中具体介绍
```

#### case 1

~~~matlab
case1,
sys=mdlDerivatives(t,x,u);
~~~

`flag=1`表示此时要计算连续状态的微分，即上面提到的$dx/dt=fc(t,x,u)$中的$dx/dt$，找到`mdlDerivatives`函数（在193行）如果设置连续状态变量个数为0，此处只需`sys=[]`;就可以了（如sfuntmpl中一样）

按我们上述讨论的那个模型，此处改成`sys=fc(t,x(1),u)`或`sys=A*x(1)+B*u` 我们这儿x(1)是连续状态变量，而x(2)是离散的，这儿只用到连续的，此时的输出sys就是微分

~~~matlab
function sys=mdlDerivatives(t,x,u)

sys = [];
~~~

#### case 2

```matlab
case2,
sys=mdlUpdate(t,x,u);
```

`flag=2`表示此时要计算下一个离散状态，即上面提到的$x(k+1)=fd(t,x,u)$，找到`mdlUpdate 函数`（在206行）它这儿`sys=[];`表示没有离散状态，我们这而可以改成`sys=fd(t,x(2),u)`或`sys=H*x(2)+G*u;`%sys即为x(k+1)

```matlab
function sys=mdlUpdate(t,x,u)

sys = [];
```

#### case 3

```matlab
case3,
sys=mdlOutputs(t,x,u);
```

`flag=3`表示此时要计算输出，即`y=fo(t,x,u)`,找到`mdlOutputs函数`（在218行），如上，如果`sys=[]`表示没有输出，我们改成`sys=fo(t,x,u)`或`sys=C*x+D*u`%sys此时为输出y

```matlab
function sys=mdlOutputs(t,x,u)
sys = [];
```

#### case 4

```matlab
case4,
sys=mdlGetTimeOfNextVarHit(t,x,u);

```

```matlab
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    % 例如，将下一个命中设定为一秒后。
sys = t + sampleTime;
```

`flag=4`表示此时要计算下一次采样的时间，只在离散采样系统中有用(即上文的`mdlInitializeSizes`中提到的ts设置ts(1)不为0)

连续系统中只需在`mdlGetTimeOfNextVarHit函数`中写上`sys=[];`这个函数主要用于变步长的设置，具体实现大家可以用edit vsfunc看vsfunc.m这个例子

#### case 9

```matlab
case9,
sys=mdlTerminate(t,x,u);
```

```matlab
function sys=mdlTerminate(t,x,u)

sys = [];
```

`flag=9`表示此时系统要结束，一般来说写上在`mdlTerminate函数`中写上`sys=[]`就可，如果你在结束时还要设置什么，就在此函数中写