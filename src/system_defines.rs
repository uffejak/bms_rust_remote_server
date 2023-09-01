//pub const PORT_OTHER: &str = "COM3";
use std::net::*;

#[cfg(feature = "config_type_1")]
pub const PORT_OTHER: &str = "COM3";
#[cfg(feature = "config_type_1")]
pub const BOARD_CONFIG: [bool; 15] = [false; 15]; // Each array entry corresponds to a sensor board, true implies mounted while false implies unmounted
#[cfg(feature = "config_type_1")]
pub const PUMP_CONFIG: bool = false;
#[cfg(feature = "config_type_1")]
pub const INVERTER_CONFIG: bool = false;

#[cfg(feature = "config_type_2")]
pub const INVERTER_CONFIG: bool = false;
#[cfg(feature = "config_type_2")]
pub const BOARD_CONFIG: [bool; 15] = [false; 15]; // Each array entry corresponds to a sensor board, true implies mounted while false implies unmounted
#[cfg(feature = "config_type_2")]
pub const PORT_OTHER: &str = "COM3";
#[cfg(feature = "config_type_2")]
pub const PUMP_CONFIG: bool = false;

pub const DEBUG_LVL_NONE:u16 = 0; //No debug text to console
pub const DEBUG_LVL_ERROR:u16 = 1; //Only errors to console
pub const DEBUG_LVL_WARNING:u16 = 2; //Only errors+warnings to console
pub const DEBUG_LVL_INFO:u16 = 3; //Only errors+warnings+info to console
pub const DEBUG_LVL_ALL:u16 = 4; //All text to console
pub const DEBUG_LVL:u16 = DEBUG_LVL_INFO; //if DEBUG_LVL >= DEBUG_LVL_... { <do stuff> } 

pub const TICKRATE_REMOTESERVER: u64 = 1;
pub const TICKRATE_PUMP: u64 = 1;
pub const TICKRATE_INVERTER: u64 = 1;
pub const TICKRATE_OTHER: u64 = 1;
pub const TICKRATE_INTERFACE: u64 = 1; 
pub const TICKRATE_COMMAND: f64 = 0.3333333;

pub const pump_a0: f32 = 6.7648e4;
pub const pump_a1: f32 = 8.9414e6;
pub const pump_a2: f32 = -2.2589e10;

pub const MAX_COM_ERROR_COUNT: u32 = 20;
pub const MAX_CHANNEL_ERROR_COUNT: u16 = 10;

pub const JENS_INVERTER_USB: &str = "/dev/tty.usbmodem144103";
pub const CHRISTIAN_INVERTER_USB: &str = "COM3";

//TODO
//pub const PC_IP_ADDRESS: &str = "127.0.0.1";
pub const REMOTE_SERVER_PORT: u16 = 5021;
pub const REMOTE_SERVER_IP: IpAddr = IpAddr::V4(Ipv4Addr::new(127, 0, 0, 1));
pub const PORT_INTERFACE: &str = "127.0.0.1:5019"; 
//pub const PORT_OF_CALL: &str = PC_IP_ADDRESS + ":5021";
//pub const PORT_CLIENT: &str = "127.0.0.1:5021"; 
pub const PORT_REMOTESERVER: &str = "127.0.0.1:5021";
pub const PORT_INVERTER: &str = "COM3";
pub const PORT_PUMP: &str = "COM4";

// Parameters for electrochemical model (See Badenhorst and Jensen et al. 2023)

pub const ISDIFFUSIONCELL: bool = false; // Set model type as full cell with forced flow (false) or diffusion cell (true)
pub const DIFFUSION_RATE: f32 = 6.3E-12;
pub const NOMINAL_ANOLYTE: f32 = 500.0;
pub const NOMINAL_CATHOLYTE: f32 = 0.0;
pub const RATE_ANOLYTE: f32 = 8.3E-1;
pub const RATE_CATHOLYTE: f32 = 7.6E-5;
pub const STACK_RESISTANCE: f32 = 1.63;
pub const CHARGE_OFFSET:f32 = -0.788;
pub const DISCHARGE_OFFSET:f32 = 0.620;

// Parameters for CUSUM-based charging control

pub const CHARGE_ENDPOINT:f32 = 60.0;
pub const CHARGE_MIDPOINT:f32 = 51.5;
pub const DISCHARGE_MIDPOINT:f32 = 31.5;
pub const DISCHARGE_ENDPOINT:f32 = 25.0;
pub const MAX_G:f32 = 50.0;
pub const MIN_G:f32 = 10.0;


// Parameters  of thermal model
pub const AMBIENT_TEMP:f32 = 25.0;
pub const TANK_TEMP:f32 = 60.0;
pub const STACK_VOLUME:f32   = 0.01;
pub const TANK_VOLUME:f32   = 0.05;

// POSSIBLE

#[derive(PartialEq, PartialOrd, Clone,Debug, Copy)]
pub enum GAS_RISK {
    NO_RISK = 0, // No gas suspected
    POSSIBLE_RISK = 1, // Possible risk of gas, but may just be pump operation. Proceed with caution.
    HIGH_RISK = 2, // High risk of gas, shut system down (overpressure with pumps off or gas detector firing)
}

#[derive(PartialEq, PartialOrd, Clone,Debug, Copy)]
pub enum TMS_STATE{
    IDLE = 0,
    TEMP_OK = 1, 
    TEMP_HIGH_ERROR = 2,
}


//parameters for simple battery model


// pub const BATTERY_1_C:f32 = 20.0; //Amps
pub const BATTERY_FIXED_CURRENT:f32 = 20.0; //Amp
pub const BATTERY_DC_VOLTAGE_MIN:f32 = 20.0; //Volts
pub const BATTERY_DC_VOLTAGE_MAX:f32 = 58.0; //Volts
// pub const BATTERY_SOC_MIN_PCT:f32 = 0.200; //%
// pub const BATTERY_SOC_MAX_PCT:f32 = 0.800; //%
// pub const BATTERY_SOC_HYSTERESIS_PCT:f32 = 0.05;

pub const HOURS_TO_SECONDS:f32 = 60.0*60.0;
pub const SECONDS_TO_HOURS:f32 = 1.0/HOURS_TO_SECONDS;
// pub const BATTERY_MAXCHARGE:f32 = BATTERY_1_C * HOURS_TO_SECONDS; //A*s
// pub const BATTERY_SOC_MIN:f32 = BATTERY_SOC_MIN_PCT * BATTERY_MAXCHARGE;
// pub const BATTERY_SOC_MAX:f32 = BATTERY_SOC_MAX_PCT * BATTERY_MAXCHARGE;
// pub const BATTERY_SOC_HYSTERESIS:f32 = BATTERY_SOC_HYSTERESIS_PCT * BATTERY_MAXCHARGE;
// pub const BATTERY_MAX_CURRENT:f32 = BATTERY_1_C;
// pub const BATTERY_MAX_POWER:f32 = BATTERY_MAX_CURRENT * BATTERY_DC_VOLTAGE_MAX;

pub const NOMINAL_POWER_CUBER:f32 = 1000.0; //Watt
// pub const POWER_SCALING_TO_BATTERY:f32 = BATTERY_MAX_POWER / NOMINAL_POWER_CUBER; //per unit


// SOC(t)     = soc_0 + integral(i_charge(t),t0,t)
// SOC_rel(t) = (1 / BATTERY_1_C) * SOC(t)
