#include "mode.h"

extern char command_From_Bluetooth; // 用于存储蓝牙接收到的命令

LOC target_loc; // 用于存储目标位置
LOC waiting_loc;
const LOC FAR_WAITING_LOC = {2, {FORWARD, LEFT}}; // 向远处送药时的等待位置
const LOC MIDDLE_WAITING_LOC = {2, {FORWARD, FORWARD}};

bool drugSet(MODE* mode)
{
    uint16_t change_state;
    if(command_From_Bluetooth == 0) // 如果没有接收到命令
    {
        return false; // 不进行任何操作
    }

    if(command_From_Bluetooth == 'M') // 如果接收到'M'命令,也就是送药到中部
    {
        waiting_loc = FAR_WAITING_LOC;
        if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET && mode->loc.n == 0)
        {
            HAL_Delay(10); // 防抖动延时
            if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET) // 再次确认状态
            {
                mode->drug = PROPEL_MODE; // 送药模式
                mode->loc.n = 0; // 位置归零
                return true;
            }
        }
    }
    else if(command_From_Bluetooth == 'F') // 如果接收到'F'命令,也就是取药到前方
    {
        waiting_loc = MIDDLE_WAITING_LOC;
        mode->drug = PROPEL_MODE; // 送药模式
        mode->loc.n = 0; // 位置归零
        return true;
    }
    return false; // 没有模式变化
}

//以下函数用于在获取下一个方向的同时更新mode
bool isEndOfWay(LOC l)
{
    if(l.n == 1)
    {
        if(l.trace[0] == LEFT || l.trace[0] == RIGHT)
        {
            return true; // 只有一个方向且是左或右，表示到达终点
        }
    }
    else if(l.n == 2)
    {
        if((l.trace[0] == FORWARD && l.trace[1] == LEFT) || 
           (l.trace[0] == FORWARD && l.trace[1] == RIGHT) )
        {
            return true; // 两个方向且是前进加左或右，表示到达终点
        }
    }
    else if (l.n == 4)
    {
        if((l.trace[0] == FORWARD && l.trace[1] == FORWARD) &&
           (l.trace[2] != FORWARD && l.trace[3] != FORWARD) &&
           (l.trace[2] != BACKWARD && l.trace[3] != BACKWARD))
        {
            return true; // 四个方向且前进两次，其他两次不是前进或后退，表示到达终点
        }
    }
    return false; // 其他情况不是终点
}
DIR DirGet(MODE* mode)
{
    // 获取下一个方向
    if(command_From_Bluetooth != 'C')//不是继续送药的指令,前往等待地点
    {
        mode->loc.trace[mode->loc.n] = waiting_loc.trace[mode->loc.n];
        mode->loc.n++;
        return mode->loc.trace[mode->loc.n - 1];
    }
    if(command_From_Bluetooth == 'C')
    {
        if(mode->loc.n == 2 && 
           (mode->loc.trace[0] == FORWARD && mode->loc.trace[1] == FORWARD))
        {
            mode->loc.trace[mode->loc.n - 1] = target_loc.trace[mode->loc.n - 1];
            return target_loc.trace[mode->loc.n - 1]; // 返回前进方向
        }

        if(mode->loc.n == 2 && 
           (mode->loc.trace[0] == FORWARD && mode->loc.trace[1] == LEFT))
        {
            mode->loc.trace[mode->loc.n - 1] = FORWARD; // 将最后一个方向设置为前进,虽然实际上我们是右转，但该车的位置是直走再直走
            return RIGHT; // 返回右转方向
        }
    }
    mode->loc.trace[mode->loc.n] = mode->dir; // 将当前方向存入位置记录
    mode->loc.n++; // 增加位置记录的数量 
	return mode->dir; // 返回当前方向
}

//用于根据位置换算位姿
float DirToTheta(DIR dir)
{
    // 将方向转换为角度
    switch(dir)
    {
        case LEFT:
            return 90.0f; // 左转对应90度
        case RIGHT:
            return -90.0f; // 右转对应-90度
        case FORWARD:
            return 0.0f; // 前进对应0度
        case BACKWARD:
            return 180.0f; // 后退对应180度
        default:
            return 0.0f; // 默认返回0度
    }
}
float LocToTheta(LOC loc)
{
    float theta = 0.0f; // 初始化角度
    // 将位置转换为角度
    for(int i = 0; i < loc.n; i++)
    {
        theta += DirToTheta(loc.trace[i]); // 累加每个方向对应的角度
    }
    return theta;
}

//以下函数用于服务发收信息中对location的处理
LOC BStringToLoc(char* str, int len)
{
    LOC loc = {0}; // 初始化位置
    loc.n = 0; // 初始位置数为0
    for(int i = 0; i < len && loc.n < 10; i++)
    {
        loc.trace[loc.n++] = str[i];
    } // 将字符串转换为位置
    return loc; // 返回转换后的位置
}