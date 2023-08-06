/****************************************************************************
 *  Copyright (C) 2020 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "sys.h"
#include "shoot.h"
#include "dbus.h"
#include "shoot.h"
#include "shoot_task.h"
#include "event_mgr.h"
#include "event.h"
#include "os_timer.h"

#include "init.h"

#include "tim.h"

#include "log.h"

struct pid_param turn_motor_param =
    {
        .p = 10.0f,
        .i = 0.3f,
        .max_out = 30000,
        .integral_limit = 10000,
};

struct pid_param fric_motor_param = 
{
    .p = 10.0f,
    .i = 0.3f,
    .max_out = 30000,
    .integral_limit = 10000,
};

static void shoot_dr16_data_update(uint32_t eventID, void *pMsgData, uint32_t timeStamp);

struct shoot shoot;
struct rc_device shoot_rc;

int32_t shoot_firction_toggle(shoot_t p_shoot);
int32_t shoot_firction_on(shoot_t p_shoot);
int32_t shoot_firction_off(shoot_t p_shoot);

void shoot_task(void const *argument)
{
    uint32_t shoot_time = 0;

    subscriber_t listSubs;

    EventSubscribeInit(&listSubs, SUBS_MODE_NORMAL);
    EventSubscribe(&listSubs, DBUS_MSG, DBUS_MSG_LEN, 3, shoot_dr16_data_update);

    rc_device_register(&shoot_rc, "Shoot RC");

    soft_timer_register((soft_timer_callback)shoot_pid_calculate, (void *)&shoot, 5);

    shoot_pid_init(&shoot, "Shoot", turn_motor_param, DEVICE_CAN2, 0x207);

    uint16_t fric_can_id[2] = {0x201, 0x202};
    shoot_fric_wheel_init(&shoot, "CANFRIC", fric_motor_param, DEVICE_CAN2, fric_can_id, 16384);

    int is_shooting = 0;

    while (1)
    {
        /* dr16 data update */
        EventMsgProcess(&listSubs, 0);

        // if (rc_device_get_state(&shoot_rc, RC_S1_MID2UP) == E_OK)
        // {
        //     shoot_firction_toggle(&shoot);
        // }
        // else if (rc_device_get_state(&shoot_rc, RC_S1_UP2MID) == E_OK)
        // {
        //     shoot_set_cmd(&shoot, SHOOT_STOP_CMD, 0);
        // }

        if (rc_device_get_state(&shoot_rc, RC_S1_UP) == E_OK)
            shoot_firction_on(&shoot);
        else
            shoot_firction_off(&shoot);

        // if (rc_device_get_state(&shoot_rc, RC_S1_MID2DOWN) == E_OK)
        // {
        //     shoot_set_cmd(&shoot, SHOOT_ONCE_CMD, 1);
        //     shoot_time = get_time_ms();
        // }

        // get side wheel level
        int16_t wheel = shoot_rc.rc_info.wheel;
        // log_i("wheel value: %d.", wheel);

        if (shoot_rc.rc_info.mouse.l || wheel > 330 && !is_shooting)
        {
            shoot_set_cmd(&shoot, SHOOT_CONTINUOUS_CMD, 0);
            is_shooting = 1;
        }
        else if ((shoot_rc.rc_info.mouse.l == 0) && wheel < 330 && is_shooting)
        {
            shoot_set_cmd(&shoot, SHOOT_ONCE_CMD, 0);
            is_shooting = 0;
        }
        fric_pid_calc(&shoot);

        // if (rc_device_get_state(&shoot_rc, RC_S2_DOWN) != E_OK)
        // {
        //     if (rc_device_get_state(&shoot_rc, RC_S1_DOWN) == E_OK)
        //     {
        //         if (get_time_ms() - shoot_time > 2500)
        //         {
        //             shoot_set_cmd(&shoot, SHOOT_CONTINUOUS_CMD, 0);
        //         }
        //     }

        //     if (rc_device_get_state(&shoot_rc, RC_S1_MID) == E_OK)
        //     {
        //         shoot_set_cmd(&shoot, SHOOT_STOP_CMD, 0);
        //     }
        // }
        osDelay(5);
    }
}

int32_t shoot_firction_toggle(shoot_t p_shoot)
{
    static uint8_t toggle = 0;
    if (toggle)
    {
        shoot_set_fric_speed(p_shoot, 1000, 1000);
        // Turn off laser
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
        turn_off_can_fric(&shoot);
    }
    else
    {
        // shoot_set_fric_speed(p_shoot, 1250, 1250);
        if (get_speed_cfg() == NOJMP_SPEED)
            shoot_set_fric_speed(p_shoot, FIRC_MAX_SPEED, FIRC_MAX_SPEED);
        else 
            shoot_set_fric_speed(p_shoot, FIRC_MAX_SPEED_PLUG, FIRC_MAX_SPEED_PLUG);


        // Turn on laser
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 1999);

        turn_on_can_fric(&shoot);
    }
    toggle = ~toggle;
    return 0;
}

int32_t shoot_firction_on(shoot_t p_shoot)
{
    // PWM
    if (get_speed_cfg() == NOJMP_SPEED)
        shoot_set_fric_speed(p_shoot, FIRC_MAX_SPEED, FIRC_MAX_SPEED);
    else 
        shoot_set_fric_speed(p_shoot, FIRC_MAX_SPEED_PLUG, FIRC_MAX_SPEED_PLUG);
    // Turn on laser
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 1999);

    // CAN
    turn_on_can_fric(&shoot);

    // Flag
    static uint8_t toggle = 1;   

    return E_OK;
}

int32_t shoot_firction_off(shoot_t p_shoot)
{
    shoot_set_fric_speed(p_shoot, 1000, 1000);
    // Turn off laser
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
    turn_off_can_fric(&shoot);

    static uint8_t toggle = 0;
    
    return E_OK;
}

struct shoot *get_shoot(void)
{
    return &shoot;
}

/**
 * @brief  subscrib dr16 event, update
 * @param
 * @retval void
 */
static void shoot_dr16_data_update(uint32_t eventID, void *pMsgData, uint32_t timeStamp)
{
    rc_device_date_update(&shoot_rc, pMsgData);
}
