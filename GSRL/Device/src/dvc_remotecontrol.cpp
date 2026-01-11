/**
 ******************************************************************************
 * @file           : dvc_remotecontrol.cpp
 * @brief          : 遥控器适配
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 GMaster
 * All rights reserved.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "dvc_remotecontrol.hpp"

RemoteControl::RemoteControl(fp32 stickDeadZone)
    : m_uartRxTimestamp(0),
      m_isConnected(false),
      m_isDecodeCompleted(false),
      m_stickDeadZone(stickDeadZone)
{
}

bool RemoteControl::isConnected()
{
    decodeRxData();
    return m_isConnected;
}

RemoteControl::SwitchEvent RemoteControl::judgeSwitchStatus(SwitchStatus currentStatus, SwitchStatus lastStatus)
{
    switch (currentStatus - lastStatus) {
        case 0:
            return SWITCH_NO_CHANGE;
        case -2:
            return SWITCH_TOGGLE_MIDDLE_UP;
        case -1:
            return SWITCH_TOGGLE_MIDDLE_DOWN;
        case 1:
            return SWITCH_TOGGLE_DOWN_MIDDLE;
        case 2:
            return SWITCH_TOGGLE_UP_MIDDLE;
        default:
            return SWITCH_NO_CHANGE;
    }
}

RemoteControl::KeyEvent RemoteControl::judgeKeyStatus(KeyStatus currentStatus, KeyStatus lastStatus)
{
    switch (currentStatus - lastStatus) {
        case 0:
            return KEY_NO_CHANGE;
        case -1:
            return KEY_TOGGLE_PRESS_RELEASE;
        case 1:
            return KEY_TOGGLE_RELEASE_PRESS;
        default:
            return KEY_NO_CHANGE;
    }
}

fp32 RemoteControl::applyStickDeadZone(fp32 stickValue)
{
    if (fabs(stickValue) < m_stickDeadZone) {
        return 0.0f;
    } else if (stickValue > 0.0f) {
        return (stickValue - m_stickDeadZone) / (1.0f - m_stickDeadZone);
    } else {
        return (stickValue + m_stickDeadZone) / (1.0f - m_stickDeadZone);
    }
}

/* Dr16RemoteControl 实现 ------------------------------------------*/

Dr16RemoteControl::Dr16RemoteControl(fp32 stickDeadZone)
    : RemoteControl(stickDeadZone),
      m_originalRxDataPointer(nullptr),
      m_rightStickX(0.0f),
      m_rightStickY(0.0f),
      m_leftStickX(0.0f),
      m_leftStickY(0.0f),
      m_scrollWheel(0.0f),
      m_rightSwitchStatus(SWITCH_MIDDLE),
      m_lastRightSwitchStatus(SWITCH_MIDDLE),
      m_rightSwitchEvent(SWITCH_EVENT_NO_UPDATE_ERROR),
      m_leftSwitchStatus(SWITCH_MIDDLE),
      m_lastLeftSwitchStatus(SWITCH_MIDDLE),
      m_leftSwitchEvent(SWITCH_EVENT_NO_UPDATE_ERROR),
      m_mouseXSpeed(0.0f),
      m_mouseYSpeed(0.0f),
      m_mouseWheelSpeed(0.0f),
      m_mouseLeftKeyStatus(KEY_RELEASE),
      m_lastMouseLeftKeyStatus(KEY_RELEASE),
      m_mouseLeftKeyEvent(KEY_EVENT_NO_UPDATE_ERROR),
      m_mouseRightKeyStatus(KEY_RELEASE),
      m_lastMouseRightKeyStatus(KEY_RELEASE),
      m_mouseRightKeyEvent(KEY_EVENT_NO_UPDATE_ERROR)
{
    // 初始化键盘按键状态数组
    for (uint8_t i = 0; i < KEY_TOTAL_NUMBER; i++) {
        m_keyboardKeyStatus[i]     = KEY_RELEASE;
        m_lastKeyboardKeyStatus[i] = KEY_RELEASE;
        m_keyboardKeyEvent[i]      = KEY_NO_CHANGE;
    }
}

/**
 * @brief 从中断中获取DR16遥控器接收数据地址，更新相关标志位
 * @param data DR16遥控器接收数据指针
 * @note 本函数不解码遥控器数据
 */
void Dr16RemoteControl::receiveRxDataFromISR(const uint8_t *data)
{
    m_originalRxDataPointer = (RawPacket *)data;
    m_uartRxTimestamp       = HAL_GetTick();
    m_isConnected           = true;
    m_isDecodeCompleted     = false;
}

/**
 * @brief 接收DR16遥控器数据后解码数据, 判断遥控器连接状态
 * @note 本函数在使用get函数获取遥控器数据时自动调用
 */
void Dr16RemoteControl::decodeRxData()
{
    // 判断遥控器连接状态，若使用的数据过时超过100ms则认为遥控器断开
    if (HAL_GetTick() - m_uartRxTimestamp > 100 || m_originalRxDataPointer == nullptr) {
        m_isConnected = false;
        return;
    }
    // 解码遥控器数据, 每次接收数据仅解码一次
    if (m_isDecodeCompleted) return;
    m_rightStickX       = (fp32)(m_originalRxDataPointer->Channel_0 - 1024) / 660.0f;
    m_rightStickY       = (fp32)(m_originalRxDataPointer->Channel_1 - 1024) / 660.0f;
    m_leftStickX        = (fp32)(m_originalRxDataPointer->Channel_2 - 1024) / 660.0f;
    m_leftStickY        = (fp32)(m_originalRxDataPointer->Channel_3 - 1024) / 660.0f;
    m_scrollWheel       = (fp32)(m_originalRxDataPointer->Channel_4 - 1024) / 660.0f;
    m_mouseXSpeed       = (fp32)(m_originalRxDataPointer->Mouse_X / 32768.0f);
    m_mouseYSpeed       = (fp32)(m_originalRxDataPointer->Mouse_Y / 32768.0f);
    m_mouseWheelSpeed   = (fp32)(m_originalRxDataPointer->Mouse_Z / 32768.0f);
    m_isDecodeCompleted = true; // 更新解码完成标志，避免重复解码
}

/**
 * @brief 更新所有按键和拨杆的跳变事件并缓存
 * @note 使用get方法获取按键状态前请先调用本函数
 * @note 建议在每个控制循环的开头调用一次，且一个控制循环内只调用一次，以保证控制循环中获取的事件状态一致
 */
void Dr16RemoteControl::updateEvent()
{
    if (m_originalRxDataPointer == nullptr) return;
    m_lastRightSwitchStatus   = m_rightSwitchStatus;
    m_rightSwitchStatus       = (SwitchStatus)m_originalRxDataPointer->Switch_2;
    m_rightSwitchEvent        = judgeSwitchStatus(m_rightSwitchStatus, m_lastRightSwitchStatus);
    m_lastLeftSwitchStatus    = m_leftSwitchStatus;
    m_leftSwitchStatus        = (SwitchStatus)m_originalRxDataPointer->Switch_1;
    m_leftSwitchEvent         = judgeSwitchStatus(m_leftSwitchStatus, m_lastLeftSwitchStatus);
    m_lastMouseLeftKeyStatus  = m_mouseLeftKeyStatus;
    m_mouseLeftKeyStatus      = (KeyStatus)m_originalRxDataPointer->Mouse_Left_Key;
    m_mouseLeftKeyEvent       = judgeKeyStatus(m_mouseLeftKeyStatus, m_lastMouseLeftKeyStatus);
    m_lastMouseRightKeyStatus = m_mouseRightKeyStatus;
    m_mouseRightKeyStatus     = (KeyStatus)m_originalRxDataPointer->Mouse_Right_Key;
    m_mouseRightKeyEvent      = judgeKeyStatus(m_mouseRightKeyStatus, m_lastMouseRightKeyStatus);
    for (uint8_t keyIndex = 0; keyIndex < KEY_TOTAL_NUMBER; keyIndex++) {
        m_lastKeyboardKeyStatus[keyIndex] = m_keyboardKeyStatus[keyIndex];
        m_keyboardKeyStatus[keyIndex]     = (KeyStatus)(m_originalRxDataPointer->Keyboard_Key >> keyIndex & 0x01);
        m_keyboardKeyEvent[keyIndex]      = judgeKeyStatus(m_keyboardKeyStatus[keyIndex], m_lastKeyboardKeyStatus[keyIndex]);
    }
}

/* ET08ARemoteControl 实现 ------------------------------------------*/

ET08ARemoteControl::ET08ARemoteControl(Config config, fp32 stickDeadZone)
    : RemoteControl(stickDeadZone),
      m_originalRxDataPointer(nullptr),
      m_protocolData{},
      m_config(config),
      m_switchSA(SWITCH_UP),
      m_switchSB(SWITCH_MIDDLE),
      m_switchSC(SWITCH_MIDDLE),
      m_switchSD(SWITCH_UP),
      m_lastSwitchSA(SWITCH_MIDDLE),
      m_lastSwitchSB(SWITCH_UP),
      m_lastSwitchSC(SWITCH_UP),
      m_lastSwitchSD(SWITCH_MIDDLE),
      m_eventSA(SWITCH_EVENT_NO_UPDATE_ERROR),
      m_eventSB(SWITCH_EVENT_NO_UPDATE_ERROR),
      m_eventSC(SWITCH_EVENT_NO_UPDATE_ERROR),
      m_eventSD(SWITCH_EVENT_NO_UPDATE_ERROR),
      m_knobLD(0.0f),
      m_knobRD(0.0f),
      m_trimmerT1(0.0f),
      m_trimmerT2(0.0f),
      m_trimmerT3(0.0f),
      m_trimmerT4(0.0f),
      m_rightStickX(0.0f),
      m_rightStickY(0.0f),
      m_leftStickX(0.0f),
      m_leftStickY(0.0f),
      m_rightSwitchStatus(SWITCH_MIDDLE),
      m_rightSwitchEvent(SWITCH_EVENT_NO_UPDATE_ERROR),
      m_leftSwitchStatus(SWITCH_MIDDLE),
      m_leftSwitchEvent(SWITCH_EVENT_NO_UPDATE_ERROR)
{
}

/**
 * @brief 从中断中获取ET08A遥控器接收数据地址，更新相关标志位
 * @param data ET08A遥控器接收数据指针
 * @note 本函数不解码遥控器数据
 */

void ET08ARemoteControl::receiveRxDataFromISR(const uint8_t *data)
{
    m_originalRxDataPointer = (ET08ARawPacket *)data;
    m_uartRxTimestamp       = HAL_GetTick();
    m_isConnected           = true;
    m_isDecodeCompleted     = false;
}

/**
 * @brief 接收ET08A遥控器数据后解码数据, 判断遥控器连接状态
 * @note 本函数在使用get函数获取遥控器数据时自动调用
 */

void ET08ARemoteControl::decodeRxData()
{
    if (HAL_GetTick() - m_uartRxTimestamp > 100 || m_originalRxDataPointer == nullptr) {
        m_isConnected = false;
        return;
    }
    if (m_isDecodeCompleted) return;

    if (m_originalRxDataPointer->startByte != 0x0F) return;

    parseET08AProtocol(m_originalRxDataPointer->data, m_protocolData);

    m_rightStickX = (m_protocolData.rightStickX - 1024) / 660.0f;
    m_rightStickY = (m_protocolData.rightStickY - 1024) / 660.0f;
    m_leftStickX  = (m_protocolData.leftStickX - 1024) / 660.0f;
    m_leftStickY  = (m_protocolData.leftStickY - 1024) / 660.0f;

    m_knobLD = (m_protocolData.knobLD - 1024) / 660.0f;
    m_knobRD = (m_protocolData.knobRD - 1024) / 660.0f;

    m_trimmerT1 = (m_protocolData.trimmerT1 - 1024) / 660.0f;
    m_trimmerT2 = (m_protocolData.trimmerT2 - 1024) / 660.0f;
    m_trimmerT3 = (m_protocolData.trimmerT3 - 1024) / 660.0f;
    m_trimmerT4 = (m_protocolData.trimmerT4 - 1024) / 660.0f;

    m_isDecodeCompleted = true;
}

/**
 * @brief 更新所有按键和拨杆的跳变事件并缓存
 * @note 使用get方法获取按键状态前请先调用本函数
 * @note 建议在每个控制循环的开头调用一次，且一个控制循环内只调用一次，以保证控制循环中获取的事件状态一致
 */

void ET08ARemoteControl::updateEvent()
{
    if (m_originalRxDataPointer == nullptr) return;

    decodeRxData();

    // SA和SD只有两个挡位
    // 更新开关

    m_lastSwitchSA = m_switchSA;
    if (m_protocolData.switchSA < 1200) {
        m_switchSA = SWITCH_UP;
    } else if (m_protocolData.switchSA > 1200) {
        m_switchSA = SWITCH_DOWN;
    }
    m_eventSA      = judgeSwitchStatus(m_switchSA, m_lastSwitchSA);
    m_lastSwitchSB = m_switchSB;
    if (m_protocolData.switchSB < 1024) {
        m_switchSB = SWITCH_UP;
    } else if (m_protocolData.switchSB > 1024) {
        m_switchSB = SWITCH_DOWN;
    } else {
        m_switchSB = SWITCH_MIDDLE;
    }
    m_eventSB      = judgeSwitchStatus(m_switchSB, m_lastSwitchSB);
    m_lastSwitchSC = m_switchSC;
    if (m_protocolData.switchSC < 1024) {
        m_switchSC = SWITCH_UP;
    } else if (m_protocolData.switchSC > 1024) {
        m_switchSC = SWITCH_DOWN;
    } else {
        m_switchSC = SWITCH_MIDDLE;
    }
    m_eventSC      = judgeSwitchStatus(m_switchSC, m_lastSwitchSC);
    m_lastSwitchSD = m_switchSD;
    if (m_protocolData.switchSD < 1200) {
        m_switchSD = SWITCH_UP;
    } else if (m_protocolData.switchSD > 1200) {
        m_switchSD = SWITCH_DOWN;
    }
    m_eventSD      = judgeSwitchStatus(m_switchSD, m_lastSwitchSD);
}

/******************************************************************************
 *                           ET08A协议辅助函数实现
 ******************************************************************************/

 /*
    @brief ET08AConfig 默认构造函数，设置默认通道映射
*/

ET08ARemoteControl::Config::Config()
    : rightStickJ1X(ET08AChannelIndex::CH_1),
      rightStickJ2Y(ET08AChannelIndex::CH_3),
      leftStickJ3Y(ET08AChannelIndex::CH_2),
      leftStickJ4X(ET08AChannelIndex::CH_4),
      switchSA(ET08AChannelIndex::CH_5),
      switchSB(ET08AChannelIndex::CH_NONE),
      switchSC(ET08AChannelIndex::CH_NONE),
      switchSD(ET08AChannelIndex::CH_6),
      knobLD(ET08AChannelIndex::CH_7),
      knobRD(ET08AChannelIndex::CH_8),
      trimmerT1(ET08AChannelIndex::CH_NONE),
      trimmerT2(ET08AChannelIndex::CH_NONE),
      trimmerT3(ET08AChannelIndex::CH_NONE),
      trimmerT4(ET08AChannelIndex::CH_NONE)
{
}

/*
    @brief 解析ET08A遥控器协议数据
    @param buffer 原始协议数据缓冲区指针
*/

void ET08ARemoteControl::parseET08AProtocol(const uint8_t *buffer, ET08AProtocolData &out) const
{
    if (buffer == nullptr) return;

    std::uint16_t temp_ch[16];

    temp_ch[0]  = ((buffer[0] | buffer[1] << 8) & 0x07FF);
    temp_ch[1]  = ((buffer[1] >> 3 | buffer[2] << 5) & 0x07FF);
    temp_ch[2]  = ((buffer[2] >> 6 | buffer[3] << 2 | buffer[4] << 10) & 0x07FF);
    temp_ch[3]  = ((buffer[4] >> 1 | buffer[5] << 7) & 0x07FF);
    temp_ch[4]  = ((buffer[5] >> 4 | buffer[6] << 4) & 0x07FF);
    temp_ch[5]  = ((buffer[6] >> 7 | buffer[7] << 1 | buffer[8] << 9) & 0x07FF);
    temp_ch[6]  = ((buffer[8] >> 2 | buffer[9] << 6) & 0x07FF);
    temp_ch[7]  = ((buffer[9] >> 5 | buffer[10] << 3) & 0x07FF);
    temp_ch[8]  = ((buffer[11] | buffer[12] << 8) & 0x07FF);
    temp_ch[9]  = ((buffer[12] >> 3 | buffer[13] << 5) & 0x07FF);
    temp_ch[10] = ((buffer[13] >> 6 | buffer[14] << 2 | buffer[15] << 10) & 0x07FF);
    temp_ch[11] = ((buffer[15] >> 1 | buffer[16] << 7) & 0x07FF);
    temp_ch[12] = ((buffer[16] >> 4 | buffer[17] << 4) & 0x07FF);
    temp_ch[13] = ((buffer[17] >> 7 | buffer[18] << 1 | buffer[19] << 9) & 0x07FF);
    temp_ch[14] = ((buffer[19] >> 2 | buffer[20] << 6) & 0x07FF);
    temp_ch[15] = ((buffer[20] >> 5 | buffer[21] << 3) & 0x07FF);

    // 根据配置映射通道数据
    auto getChannelValue = [&temp_ch](ET08AChannelIndex index) -> uint16_t {
        return (index == ET08AChannelIndex::CH_NONE) ? 1024 : temp_ch[static_cast<uint8_t>(index)];
    };

    out.rightStickX = getChannelValue(m_config.rightStickJ1X);
    out.rightStickY = getChannelValue(m_config.rightStickJ2Y);
    out.leftStickX  = getChannelValue(m_config.leftStickJ4X);
    out.leftStickY  = getChannelValue(m_config.leftStickJ3Y);

    out.switchSA = getChannelValue(m_config.switchSA);
    out.switchSB = getChannelValue(m_config.switchSB);
    out.switchSC = getChannelValue(m_config.switchSC);
    out.switchSD = getChannelValue(m_config.switchSD);

    out.knobLD = getChannelValue(m_config.knobLD);
    out.knobRD = getChannelValue(m_config.knobRD);

    out.trimmerT1 = getChannelValue(m_config.trimmerT1);
    out.trimmerT2 = getChannelValue(m_config.trimmerT2);
    out.trimmerT3 = getChannelValue(m_config.trimmerT3);
    out.trimmerT4 = getChannelValue(m_config.trimmerT4);
}
