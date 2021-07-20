#include "Copter.h"

#if LOGGING_ENABLED == ENABLED


//%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%

struct PACKED log_Vz {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float vz;
};

struct PACKED log_RPY_rate {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t imu_instance;
    float gyr_x, gyr_y, gyr_z;
};

struct PACKED log_RPY_rate_IMU {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float gyr_x, gyr_y, gyr_z;
};

struct PACKED log_RP {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    int16_t roll, pitch;
};

struct PACKED log_PWM {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t C1,C2,C3,C4;
};

struct PACKED log_Loop {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t num_loops, num_long_run;
    uint32_t  max_time, min_time, avg_time;
};

struct PACKED log_TaskInfo {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint16_t min_time_us;
    uint16_t max_time_us;
    uint32_t elapsed_time_us;
    uint32_t tick_count;
    uint16_t slip_count;
    uint16_t overrun_count;
};


void Copter::Log_Write_Vertical_Speed(){

     uint8_t instance =0;
     //AP_GPS *oggetto = AP_GPS::get_singleton();
    // Vector3f vec = oggetto.velocity(instance);
    // AP_GPS oggetto = AP_GPS::AP_GPS();

    //AP_GPS ogg = copter.gps;
   struct log_Vz pkt = {
            LOG_PACKET_HEADER_INIT(LOG_Vz_MSG),
            time_us  : AP_HAL::micros64(),
            vz  : AP::gps().velocity(instance).z
        };
        logger.WriteBlock(&pkt, sizeof(pkt));


}

void Copter::Log_Write_RPY_Rate(uint8_t imu){

    // uint8_t imu_instance = 0; log_IMU

   struct log_RPY_rate pkt = {
            LOG_PACKET_HEADER_INIT(LOG_RPY_RATE_MSG),
            time_us  : AP_HAL::micros64(),
			imu_instance : imu,
            gyr_x : AP::ins().get_gyro(imu).x,
            gyr_y : AP::ins().get_gyro(imu).y,
            gyr_z : AP::ins().get_gyro(imu).z
        };
        logger.WriteBlock(&pkt, sizeof(pkt));

}

void Copter::Log_Write_RPY_Rate(){



   struct log_RPY_rate_IMU pkt = {
            LOG_PACKET_HEADER_INIT(LOG_RPY_RATE_IMU_MSG),
            time_us  : AP_HAL::micros64(),
            gyr_x : AP::ins().get_gyro(1).x,
            gyr_y : AP::ins().get_gyro(1).y,
            gyr_z : AP::ins().get_gyro(1).z
        };
        logger.WriteBlock(&pkt, sizeof(pkt));

}

void Copter::Log_Write_RP(){

   struct log_RP pkt = {
            LOG_PACKET_HEADER_INIT(LOG_RP_MSG),
            time_us  : AP_HAL::micros64(),
            roll : (int16_t)AP::ahrs().roll_sensor ,
            pitch : (int16_t)AP::ahrs().pitch_sensor
        };
        logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::Log_Write_PWM(){

   struct log_PWM pkt = {
            LOG_PACKET_HEADER_INIT(LOG_PWM_MSG),
            time_us  : AP_HAL::micros64(),
            C1 : hal.rcout->read(0),
            C2 : hal.rcout->read(1),
            C3 : hal.rcout->read(2),
            C4 : hal.rcout->read(3),
        };
        logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter::Log_Write_Loop(){

   struct log_Loop pkt = {
            LOG_PACKET_HEADER_INIT(LOG_LOOP_MSG),
            time_us  : AP_HAL::micros64(),
            num_loops: AP::scheduler().perf_info.get_num_loops(),
            num_long_run: AP::scheduler().perf_info.get_num_long_running(),
            max_time : AP::scheduler().perf_info.get_max_time(),
            min_time : AP::scheduler().perf_info.get_min_time(),
            avg_time :AP::scheduler().perf_info.get_avg_time(),
        };
        logger.WriteBlock(&pkt, sizeof(pkt));
}

void Copter :: Log_Write_Task_Info(uint8_t task_index){

    const AP::PerfInfo::TaskInfo* ti = AP::scheduler().perf_info.get_task_info(task_index);
   // GCS_SEND_TEXT(MAV_SEVERITY_WARNING,"n tick %d", (int)ti->max_time_us);


   struct log_TaskInfo pkt = {
              LOG_PACKET_HEADER_INIT(LOG_LOOP_INFO_MSG),
               time_us  : AP_HAL::micros64(),
               min_time_us : ti->min_time_us ,
                max_time_us : ti->max_time_us,
                elapsed_time_us : ti->elapsed_time_us,
                tick_count : ti->tick_count,
                slip_count : ti->slip_count,
                overrun_count : ti->overrun_count,
            };

            logger.WriteBlock(&pkt, sizeof(pkt));

}

//%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%

// Code to Write and Read packets from AP_Logger log memory
// Code to interact with the user to dump or erase logs

struct PACKED log_Control_Tuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    throttle_in;
    float    angle_boost;
    float    throttle_out;
    float    throttle_hover;
    float    desired_alt;
    float    inav_alt;
    int32_t  baro_alt;
    float    desired_rangefinder_alt;
    float    rangefinder_alt;
    float    terr_alt;
    int16_t  target_climb_rate;
    int16_t  climb_rate;
};

// Write a control tuning packet
void Copter::Log_Write_Control_Tuning()
{
    // get terrain altitude
    float terr_alt = 0.0f;
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    if (!terrain.height_above_terrain(terr_alt, true)) {
        terr_alt = logger.quiet_nan();
    }
#endif
    float des_alt_m = 0.0f;
    int16_t target_climb_rate_cms = 0;
    if (!flightmode->has_manual_throttle()) {
        des_alt_m = pos_control->get_pos_target_z_cm() / 100.0f;
        target_climb_rate_cms = pos_control->get_vel_target_z_cms();
    }

    // get surface tracking alts
    float desired_rangefinder_alt;
    if (!surface_tracking.get_target_dist_for_logging(desired_rangefinder_alt)) {
        desired_rangefinder_alt = AP::logger().quiet_nan();
    }

    struct log_Control_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_CONTROL_TUNING_MSG),
        time_us             : AP_HAL::micros64(),
        throttle_in         : attitude_control->get_throttle_in(),
        angle_boost         : attitude_control->angle_boost(),
        throttle_out        : motors->get_throttle(),
        throttle_hover      : motors->get_throttle_hover(),
        desired_alt         : des_alt_m,
        inav_alt            : inertial_nav.get_altitude() / 100.0f,
        baro_alt            : baro_alt,
        desired_rangefinder_alt : desired_rangefinder_alt,
        rangefinder_alt     : surface_tracking.get_dist_for_logging(),
        terr_alt            : terr_alt,
        target_climb_rate   : target_climb_rate_cms,
        climb_rate          : int16_t(inertial_nav.get_velocity_z()) // float -> int16_t
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

// Write an attitude packet
void Copter::Log_Write_Attitude()
{
    Vector3f targets = attitude_control->get_att_target_euler_cd();
    targets.z = wrap_360_cd(targets.z);
    ahrs.Write_Attitude(targets);
    ahrs_view->Write_Rate(*motors, *attitude_control, *pos_control);
    if (should_log(MASK_LOG_PID)) {
        logger.Write_PID(LOG_PIDR_MSG, attitude_control->get_rate_roll_pid().get_pid_info());
        logger.Write_PID(LOG_PIDP_MSG, attitude_control->get_rate_pitch_pid().get_pid_info());
        logger.Write_PID(LOG_PIDY_MSG, attitude_control->get_rate_yaw_pid().get_pid_info());
        logger.Write_PID(LOG_PIDA_MSG, pos_control->get_accel_z_pid().get_pid_info() );
        if (should_log(MASK_LOG_NTUN) && (flightmode->requires_GPS() || landing_with_GPS())) {
            logger.Write_PID(LOG_PIDN_MSG, pos_control->get_vel_xy_pid().get_pid_info_x());
            logger.Write_PID(LOG_PIDE_MSG, pos_control->get_vel_xy_pid().get_pid_info_y());
        }
    }
}

// Write an EKF and POS packet
void Copter::Log_Write_EKF_POS()
{
    AP::ahrs_navekf().Log_Write();
    ahrs.Write_AHRS2();
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    sitl.Log_Write_SIMSTATE();
#endif
    ahrs.Write_POS();
}

struct PACKED log_MotBatt {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float   lift_max;
    float   bat_volt;
    float   bat_res;
    float   th_limit;
};

// Write an rate packet
void Copter::Log_Write_MotBatt()
{
#if FRAME_CONFIG != HELI_FRAME
    struct log_MotBatt pkt_mot = {
        LOG_PACKET_HEADER_INIT(LOG_MOTBATT_MSG),
        time_us         : AP_HAL::micros64(),
        lift_max        : (float)(motors->get_lift_max()),
        bat_volt        : (float)(motors->get_batt_voltage_filt()),
        bat_res         : (float)(battery.get_resistance()),
        th_limit        : (float)(motors->get_throttle_limit())
    };
    logger.WriteBlock(&pkt_mot, sizeof(pkt_mot));
#endif
}

struct PACKED log_Data_Int16t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    int16_t data_value;
};

// Write an int16_t data packet
UNUSED_FUNCTION
void Copter::Log_Write_Data(LogDataID id, int16_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Int16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT16_MSG),
            time_us     : AP_HAL::micros64(),
            id          : (uint8_t)id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt16t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    uint16_t data_value;
};

// Write an uint16_t data packet
UNUSED_FUNCTION 
void Copter::Log_Write_Data(LogDataID id, uint16_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_UInt16t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT16_MSG),
            time_us     : AP_HAL::micros64(),
            id          : (uint8_t)id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Int32t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    int32_t data_value;
};

// Write an int32_t data packet
void Copter::Log_Write_Data(LogDataID id, int32_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Int32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_INT32_MSG),
            time_us  : AP_HAL::micros64(),
            id          : (uint8_t)id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_UInt32t {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    uint32_t data_value;
};

// Write a uint32_t data packet
void Copter::Log_Write_Data(LogDataID id, uint32_t value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_UInt32t pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_UINT32_MSG),
            time_us     : AP_HAL::micros64(),
            id          : (uint8_t)id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_Data_Float {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t id;
    float data_value;
};

// Write a float data packet
UNUSED_FUNCTION
void Copter::Log_Write_Data(LogDataID id, float value)
{
    if (should_log(MASK_LOG_ANY)) {
        struct log_Data_Float pkt = {
            LOG_PACKET_HEADER_INIT(LOG_DATA_FLOAT_MSG),
            time_us     : AP_HAL::micros64(),
            id          : (uint8_t)id,
            data_value  : value
        };
        logger.WriteCriticalBlock(&pkt, sizeof(pkt));
    }
}

struct PACKED log_ParameterTuning {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  parameter;     // parameter we are tuning, e.g. 39 is CH6_CIRCLE_RATE
    float    tuning_value;  // normalized value used inside tuning() function
    float    tuning_min;    // tuning minimum value
    float    tuning_max;    // tuning maximum value
};

void Copter::Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, float tune_min, float tune_max)
{
    struct log_ParameterTuning pkt_tune = {
        LOG_PACKET_HEADER_INIT(LOG_PARAMTUNE_MSG),
        time_us        : AP_HAL::micros64(),
        parameter      : param,
        tuning_value   : tuning_val,
        tuning_min     : tune_min,
        tuning_max     : tune_max
    };

    logger.WriteBlock(&pkt_tune, sizeof(pkt_tune));
}

// logs when baro or compass becomes unhealthy
void Copter::Log_Sensor_Health()
{
    // check baro
    if (sensor_health.baro != barometer.healthy()) {
        sensor_health.baro = barometer.healthy();
        AP::logger().Write_Error(LogErrorSubsystem::BARO,
                                 (sensor_health.baro ? LogErrorCode::ERROR_RESOLVED : LogErrorCode::UNHEALTHY));
    }

    // check compass
    if (sensor_health.compass != compass.healthy()) {
        sensor_health.compass = compass.healthy();
        AP::logger().Write_Error(LogErrorSubsystem::COMPASS, (sensor_health.compass ? LogErrorCode::ERROR_RESOLVED : LogErrorCode::UNHEALTHY));
    }

    // check primary GPS
    if (sensor_health.primary_gps != gps.primary_sensor()) {
        sensor_health.primary_gps = gps.primary_sensor();
        AP::logger().Write_Event(LogEvent::GPS_PRIMARY_CHANGED);
    }
}

struct PACKED log_SysIdD {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    waveform_time;
    float    waveform_sample;
    float    waveform_freq;
    float    angle_x;
    float    angle_y;
    float    angle_z;
    float    accel_x;
    float    accel_y;
    float    accel_z;
};

// Write an rate packet
void Copter::Log_Write_SysID_Data(float waveform_time, float waveform_sample, float waveform_freq, float angle_x, float angle_y, float angle_z, float accel_x, float accel_y, float accel_z)
{
#if MODE_SYSTEMID_ENABLED == ENABLED
    struct log_SysIdD pkt_sidd = {
        LOG_PACKET_HEADER_INIT(LOG_SYSIDD_MSG),
        time_us         : AP_HAL::micros64(),
        waveform_time   : waveform_time,
        waveform_sample : waveform_sample,
        waveform_freq   : waveform_freq,
        angle_x         : angle_x,
        angle_y         : angle_y,
        angle_z         : angle_z,
        accel_x         : accel_x,
        accel_y         : accel_y,
        accel_z         : accel_z
    };
    logger.WriteBlock(&pkt_sidd, sizeof(pkt_sidd));
#endif
}

struct PACKED log_SysIdS {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  systemID_axis;
    float    waveform_magnitude;
    float    frequency_start;
    float    frequency_stop;
    float    time_fade_in;
    float    time_const_freq;
    float    time_record;
    float    time_fade_out;
};

// Write an rate packet
void Copter::Log_Write_SysID_Setup(uint8_t systemID_axis, float waveform_magnitude, float frequency_start, float frequency_stop, float time_fade_in, float time_const_freq, float time_record, float time_fade_out)
{
#if MODE_SYSTEMID_ENABLED == ENABLED
    struct log_SysIdS pkt_sids = {
        LOG_PACKET_HEADER_INIT(LOG_SYSIDS_MSG),
        time_us             : AP_HAL::micros64(),
        systemID_axis       : systemID_axis,
        waveform_magnitude  : waveform_magnitude,
        frequency_start     : frequency_start,
        frequency_stop      : frequency_stop,
        time_fade_in        : time_fade_in,
        time_const_freq     : time_const_freq,
        time_record         : time_record,
        time_fade_out       : time_fade_out
    };
    logger.WriteBlock(&pkt_sids, sizeof(pkt_sids));
#endif
}

#if FRAME_CONFIG == HELI_FRAME
struct PACKED log_Heli {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float    desired_rotor_speed;
    float    main_rotor_speed;
    float    governor_output;
    float    control_output;
};

// Write an helicopter packet
void Copter::Log_Write_Heli()
{
    struct log_Heli pkt_heli = {
        LOG_PACKET_HEADER_INIT(LOG_HELI_MSG),
        time_us                 : AP_HAL::micros64(),
        desired_rotor_speed     : motors->get_desired_rotor_speed(),
        main_rotor_speed        : motors->get_main_rotor_speed(),
        governor_output         : motors->get_governor_output(),
        control_output          : motors->get_control_output(),
    };
    logger.WriteBlock(&pkt_heli, sizeof(pkt_heli));
}
#endif

// guided target logging
struct PACKED log_GuidedTarget {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t type;
    float pos_target_x;
    float pos_target_y;
    float pos_target_z;
    float vel_target_x;
    float vel_target_y;
    float vel_target_z;
};

// Write a Guided mode target
// pos_target is lat, lon, alt OR offset from ekf origin in cm OR roll, pitch, yaw target in centi-degrees
// vel_target is cm/s
void Copter::Log_Write_GuidedTarget(ModeGuided::SubMode target_type, const Vector3f& pos_target, const Vector3f& vel_target)
{
    struct log_GuidedTarget pkt = {
        LOG_PACKET_HEADER_INIT(LOG_GUIDEDTARGET_MSG),
        time_us         : AP_HAL::micros64(),
        type            : (uint8_t)target_type,
        pos_target_x    : pos_target.x,
        pos_target_y    : pos_target.y,
        pos_target_z    : pos_target.z,
        vel_target_x    : vel_target.x,
        vel_target_y    : vel_target.y,
        vel_target_z    : vel_target.z
    };
    logger.WriteBlock(&pkt, sizeof(pkt));
}

// type and unit information can be found in  "GPS"
// libraries/AP_Logger/Logstructure.h; search for "log_Units" for
// units and "Format characters" for field type information
const struct LogStructure Copter::log_structure[] = {
    LOG_COMMON_STRUCTURES,
    
// @LoggerMessage: PTUN
// @Description: Parameter Tuning information
// @URL: https://ardupilot.org/copter/docs/tuning.html#in-flight-tuning
// @Field: TimeUS: Time since system startup
// @Field: Param: Parameter being tuned
// @Field: TunVal: Normalized value used inside tuning() function
// @Field: TunMin: Tuning minimum limit
// @Field: TunMax: Tuning maximum limit

    { LOG_PARAMTUNE_MSG, sizeof(log_ParameterTuning),
      "PTUN", "QBfff",         "TimeUS,Param,TunVal,TunMin,TunMax", "s----", "F----" },

// @LoggerMessage: CTUN
// @Description: Control Tuning information
// @Field: TimeUS: Time since system startup
// @Field: ThI: throttle input
// @Field: ABst: angle boost
// @Field: ThO: throttle output
// @Field: ThH: calculated hover throttle
// @Field: DAlt: desired altitude
// @Field: Alt: achieved altitude
// @Field: BAlt: barometric altitude
// @Field: DSAlt: desired rangefinder altitude
// @Field: SAlt: achieved rangefinder altitude
// @Field: TAlt: terrain altitude
// @Field: DCRt: desired climb rate
// @Field: CRt: climb rate

// @LoggerMessage: D16
// @Description: Generic 16-bit-signed-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: DU16
// @Description: Generic 16-bit-unsigned-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: D32
// @Description: Generic 32-bit-signed-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: DFLT
// @Description: Generic float storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

// @LoggerMessage: DU32
// @Description: Generic 32-bit-unsigned-integer storage
// @Field: TimeUS: Time since system startup
// @Field: Id: Data type identifier
// @Field: Value: Value

    { LOG_CONTROL_TUNING_MSG, sizeof(log_Control_Tuning),
      "CTUN", "Qffffffefffhh", "TimeUS,ThI,ABst,ThO,ThH,DAlt,Alt,BAlt,DSAlt,SAlt,TAlt,DCRt,CRt", "s----mmmmmmnn", "F----00B000BB" },
    
// @LoggerMessage: MOTB
// @Description: Battery information
// @Field: TimeUS: Time since system startup
// @Field: LiftMax: Maximum motor compensation gain
// @Field: BatVolt: Ratio betwen detected battery voltage and maximum battery voltage
// @Field: BatRes: Estimated battery resistance
// @Field: ThLimit: Throttle limit set due to battery current limitations

    { LOG_MOTBATT_MSG, sizeof(log_MotBatt),
      "MOTB", "Qffff",  "TimeUS,LiftMax,BatVolt,BatRes,ThLimit", "s-vw-", "F-00-" },
    { LOG_DATA_INT16_MSG, sizeof(log_Data_Int16t),         
      "D16",   "QBh",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_UINT16_MSG, sizeof(log_Data_UInt16t),         
      "DU16",  "QBH",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_INT32_MSG, sizeof(log_Data_Int32t),         
      "D32",   "QBi",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_UINT32_MSG, sizeof(log_Data_UInt32t),         
      "DU32",  "QBI",         "TimeUS,Id,Value", "s--", "F--" },
    { LOG_DATA_FLOAT_MSG, sizeof(log_Data_Float),         
      "DFLT",  "QBf",         "TimeUS,Id,Value", "s--", "F--" },
    
// @LoggerMessage: HELI
// @Description: Helicopter related messages 
// @Field: TimeUS: Time since system startup
// @Field: DRRPM: Desired rotor speed
// @Field: ERRPM: Estimated rotor speed
// @Field: Gov: Governor Output
// @Field: Throt: Throttle output
#if FRAME_CONFIG == HELI_FRAME
    { LOG_HELI_MSG, sizeof(log_Heli),
      "HELI",  "Qffff",        "TimeUS,DRRPM,ERRPM,Gov,Throt", "s----", "F----" },
#endif

// @LoggerMessage: SIDD
// @Description: System ID data
// @Field: TimeUS: Time since system startup
// @Field: Time: Time reference for waveform
// @Field: Targ: Current waveform sample
// @Field: F: Instantaneous waveform frequency
// @Field: Gx: Delta angle, X-Axis
// @Field: Gy: Delta angle, Y-Axis
// @Field: Gz: Delta angle, Z-Axis
// @Field: Ax: Delta velocity, X-Axis
// @Field: Ay: Delta velocity, Y-Axis
// @Field: Az: Delta velocity, Z-Axis

    { LOG_SYSIDD_MSG, sizeof(log_SysIdD),
      "SIDD", "Qfffffffff",  "TimeUS,Time,Targ,F,Gx,Gy,Gz,Ax,Ay,Az", "ss-zkkkooo", "F---------" },

// @LoggerMessage: SIDS
// @Description: System ID settings
// @Field: TimeUS: Time since system startup
// @Field: Ax: The axis which is being excited
// @Field: Mag: Magnitude of the chirp waveform
// @Field: FSt: Frequency at the start of chirp
// @Field: FSp: Frequency at the end of chirp
// @Field: TFin: Time to reach maximum amplitude of chirp
// @Field: TC: Time at constant frequency before chirp starts
// @Field: TR: Time taken to complete chirp waveform
// @Field: TFout: Time to reach zero amplitude after chirp finishes

    { LOG_SYSIDS_MSG, sizeof(log_SysIdS),
      "SIDS", "QBfffffff",  "TimeUS,Ax,Mag,FSt,FSp,TFin,TC,TR,TFout", "s--ssssss", "F--------" },

// @LoggerMessage: GUID
// @Description: Guided mode target information
// @Field: TimeUS: Time since system startup
// @Field: Type: Type of guided mode
// @Field: pX: Target position, X-Axis
// @Field: pY: Target position, Y-Axis
// @Field: pZ: Target position, Z-Axis
// @Field: vX: Target velocity, X-Axis
// @Field: vY: Target velocity, Y-Axis
// @Field: vZ: Target velocity, Z-Axis

    { LOG_GUIDEDTARGET_MSG, sizeof(log_GuidedTarget),
      "GUID",  "QBffffff",    "TimeUS,Type,pX,pY,pZ,vX,vY,vZ", "s-mmmnnn", "F-BBBBBB" },
//%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%
      { LOG_Vz_MSG, sizeof(log_Vz),
        "Vz",  "Qf",    "TimeUS,Vz", "sn", "F0" },

      { LOG_RPY_RATE_MSG, sizeof(log_RPY_rate),
          "RPYR",  "QBfff",    "TimeUS,Imu,GyrX,GyrY,GyrZ", "s#EEE", "F-000" },

      { LOG_RPY_RATE_IMU_MSG, sizeof(log_RPY_rate),
              "RPYR",  "Qfff",    "TimeUS,GyrX,GyrY,GyrZ", "sEEE", "F000" },

      { LOG_RP_MSG, sizeof(log_RP),
              "RP",  "Qcc",    "TimeUS,Roll,Pitch", "sdd", "FBB" },

      { LOG_PWM_MSG, sizeof(log_PWM),
                      "PWM",  "QHHHH",    "TimeUS,C1,C2,C3,C4", "sYYYY", "F----" },

      { LOG_LOOP_MSG, sizeof(log_Loop),
                    "LOOP",  "QHHIII",    "TimeUS, num_loops, num_long_run, max_time, min_time, avg_time", "s-----", "F-----" },

      //{ LOG_LOOP_INFO_MSG, sizeof(log_TaskInfo),
       //         "TASL",  "QHHIIHH",    "TimeUS, min_time_us, max_time_us, elapsed_time_us, tick_count, slip_count, overrun_count", "s------", "F------" },
      //{ LOG_VZ_INFO_MSG, sizeof(log_TaskInfo),
        //         "TASV",  "QHHIIHH",    "TimeUS, min_time_us, max_time_us, elapsed_time_us, tick_count, slip_count, overrun_count", "s------", "F------" },
     // { LOG_RP_INFO_MSG, sizeof(log_TaskInfo),
                 // "TASY",  "QHHIIHH",    "TimeUS, min_time_us, max_time_us, elapsed_time_us, tick_count, slip_count, overrun_count", "s------", "F------" },
     // { LOG_RPYR_INFO_MSG, sizeof(log_TaskInfo),
             //    "TASR",  "QHHIIHH",    "TimeUS, min_time_us, max_time_us, elapsed_time_us, tick_count, slip_count, overrun_count", "s------", "F------" },
     // { LOG_PWM_INFO_MSG, sizeof(log_TaskInfo),
                // "TASW",  "QHHIIHH",    "TimeUS, min_time_us, max_time_us, elapsed_time_us, tick_count, slip_count, overrun_count", "s------", "F------" },


//%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%
};

void Copter::Log_Write_Vehicle_Startup_Messages()
{
    // only 200(?) bytes are guaranteed by AP_Logger
    char frame_and_type_string[30];
    copter.motors->get_frame_and_type_string(frame_and_type_string, ARRAY_SIZE(frame_and_type_string));
    logger.Write_MessageF("%s", frame_and_type_string);
    logger.Write_Mode((uint8_t)flightmode->mode_number(), control_mode_reason);
    ahrs.Log_Write_Home_And_Origin();
    gps.Write_AP_Logger_Log_Startup_messages();
}

void Copter::log_init(void)
{
    logger.Init(log_structure, ARRAY_SIZE(log_structure));
}

#else // LOGGING_ENABLED

void Copter::Log_Write_Control_Tuning() {}
void Copter::Log_Write_Attitude(void) {}
void Copter::Log_Write_EKF_POS() {}
void Copter::Log_Write_MotBatt() {}
void Copter::Log_Write_Data(LogDataID id, int32_t value) {}
void Copter::Log_Write_Data(LogDataID id, uint32_t value) {}
void Copter::Log_Write_Data(LogDataID id, int16_t value) {}
void Copter::Log_Write_Data(LogDataID id, uint16_t value) {}
void Copter::Log_Write_Data(LogDataID id, float value) {}
void Copter::Log_Write_Parameter_Tuning(uint8_t param, float tuning_val, float tune_min, float tune_max) {}
void Copter::Log_Sensor_Health() {}
void Copter::Log_Write_GuidedTarget(ModeGuided::SubMode target_type, const Vector3f& pos_target, const Vector3f& vel_target) {}
void Copter::Log_Write_SysID_Setup(uint8_t systemID_axis, float waveform_magnitude, float frequency_start, float frequency_stop, float time_fade_in, float time_const_freq, float time_record, float time_fade_out) {}
void Copter::Log_Write_SysID_Data(float waveform_time, float waveform_sample, float waveform_freq, float angle_x, float angle_y, float angle_z, float accel_x, float accel_y, float accel_z) {}
void Copter::Log_Write_Vehicle_Startup_Messages() {}

#if FRAME_CONFIG == HELI_FRAME
void Copter::Log_Write_Heli() {}
#endif

void Copter::log_init(void) {}

#endif // LOGGING_ENABLED
