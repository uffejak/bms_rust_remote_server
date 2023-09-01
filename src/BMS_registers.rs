#![feature(inline_const_pat)]
#![allow(non_snake_case)] 
#![allow(non_camel_case_types)]
#![allow(non_upper_case_globals)]

use rodbus::*;
use rodbus::server::*;

use std::sync::Mutex;


#[allow(non_snake_case)] 
#[allow(non_camel_case_types)]
pub const Reg_Ctrl_BMS:u16		        = 100; //#Control of the battery (charge or discharge)
pub const Reg_SOC:u16					= 101; //#State of charge of the battery
pub const Reg_P_Ref:u16					= 102; //#Reference for charge level (positive = discharging from battery to grid)
pub const Reg_Gauss_Volt:u16			= 103; //#Gaussian charge voltage on surface of tank (UNUSED)
pub const Reg_SOC_Gauss:u16				= 104; //#State of charge estimation based on Gaussian charge voltage (UNUSED)
pub const Reg_BMS_Error:u16				= 105; //#Errors from BMS (0= no error)
pub const Reg_Stack_Current:u16			= 106; //#Current in the stack of the CuRFB
pub const Reg_Stack_Voltage:u16			= 107; //#Voltage in the stack of the CuRFB
pub const Reg_Chlorine:u16				= 108; //#0 indicates no detected risk, 50 indicates some risk, 100 indicates high risk and shutdown
pub const Reg_Temp_Anolyte_Tank:u16			= 109; //#Temperature in the anolyte tank
pub const Reg_Temp_Anolyte_In:u16		= 110; //#Temperature at the entrance of the stack
pub const Reg_Flow_Anolyte:u16			= 111; //#Flow of the anolyte circuit
pub const Reg_Pressure_Anolyte_In:u16		= 112; //#Pressure in the anolyte circuit
pub const Reg_Speed_Anolyte:u16			= 113; //#Pump speed for the anolyte circuit
pub const Reg_Temp_Catholyte_Tank:u16		= 114; //#Temperature in the catholyte tank
pub const Reg_Temp_Catholyte_In:u16		= 115; //#Temperature at the entrance of the stack
pub const Reg_Flow_Catholyte:u16		= 116; //#Flow of the catholyte circuit
pub const Reg_Pressure_Catholyte_In:u16	= 117; //#Pressure in the catholyte circuit
pub const Reg_Speed_Catholyte:u16		= 118; //#Pump for the catholyte circuit
pub const Reg_DC_DC_On:u16				= 119; //#DC/DC converter (on/off)
pub const Reg_DC_DC_Current:u16			= 120; //#Charging/discharging current in DC/DC converter
pub const Reg_DC_DC_Voltage:u16			= 121; //#Charging voltage/cell voltage in DC/DC converter
pub const Reg_DC_DC_Power:u16			= 122; //#Charging power in DC/DC converter (positive = discharing)
pub const Reg_DC_DC_Temp:u16			= 123; //#Internal temperature in DC/DC converter
pub const Reg_DC_DC_Ambient_Temp:u16	= 124; //#Ambient temperature in DC/DC converter
pub const Reg_DC_DC_Errors:u16			= 125; //#Errors from DC/DC converter (0 = no error)
pub const Reg_AC_DC_On:u16				= 126; //#AC/DC converter (on/off)
pub const Reg_AC_DC_Grid_Current:u16	= 127; //#AC/DC converter current (positive = discharging)
pub const Reg_AC_DC_Grid_Voltage:u16	= 128; //#Grid voltage in AC/DC converter (phase to phase)
pub const Reg_AC_DC_Grid_Power:u16		= 129; //#Grid power in AC/DC converter (positive = output)
pub const Reg_DC_Link_Voltage:u16		= 130; //#DC link voltage between AC/DC and DC/DC converters
pub const Reg_AC_DC_Temp:u16			= 131; //#Internal temperature in AC/DC converter
pub const Reg_AC_DC_Ambient_Temp:u16	= 132; //#Ambient temperature in AC/DC converter
pub const Reg_AC_DC_Errors:u16			= 133; //#Errors from AC/DC converter (0 = no error)
pub const Reg_Speed_Thermal:u16			= 134; //#Pump speed for thermal circuit
pub const Reg_T_Thermal_Tank:u16		= 135; //#Temperature in the thermal tank
pub const Reg_Simulation_Mode:u16		= 136; //#0=Live version on real HW, 1=Simulation level 1 (forward Euler, 1. order)
pub const Reg_Time_Contraction:u16		= 137; //#Multiplier for time in simulation (speed up factor), reduces accuracy of simulation
pub const Reg_Simulation_Clock:u16		= 138; //#Internal simulation in seconds
pub const Reg_Temp_Thermal_Catholyte:u16= 139; //#Temperature of the heat exchanger in the catholyte circuit
pub const Reg_Temp_Thermal_Anolyte:u16  = 140; //#Temperature of the heat exchanger in the anolyte circuit
pub const Reg_Pressure_Anolyte_Out:u16  = 141; //#Pressure in the anolyte circuit after the stack
pub const Reg_Pressure_Catholyte_Out:u16= 142; //#Pressure in the catholyte circuit after the stack
pub const Reg_Temp_Anolyte_Out:u16      = 143; //#Temperature in the anolyte circuit after the stack
pub const Reg_Temp_Catholyte_Out:u16    = 144; //#catholyte_temp_out
pub const Reg_State_Bms:u16           = 145; // #BMS operating state (charging, discharging, standby, reset, or fatal error)
pub const Reg_rs_est:u16                = 146; //# serial resistance * 1000 
pub const Reg_rp_est:u16                = 147; //
pub const Reg_cp_hi_est:u16             = 148; //# cp >> 15
pub const Reg_cp_lo_est:u16             = 149; //# cp & ox7fff
pub const Reg_ocv_est:u16               = 150; //# ocv * 100

pub const Reg_Inverter_Command:u16      = 500;
pub const Reg_Inverter_State:u16        = 501;
pub const Reg_Inverter_Charger_Error:u16= 502;
pub const Reg_Inverter_AC_Error:u16     = 503;
pub const Reg_DC_Current:u16            = 504; //in Amps 
pub const Reg_Charging_Direction:u16    = 505; // (0 = charging battery) , (1 = discharging battery)
pub const Reg_DC_Voltage:u16            = 506;
pub const Reg_DC_Power:u16              = 507;
pub const Reg_AC_Voltage:u16            = 508;
pub const Reg_AC_Power:u16              = 509;
pub const Reg_Inverter_Temp:u16         = 510;
pub const Reg_DCDC_Temp:u16             = 511;
pub const Reg_DCLink_Voltage:u16        = 512; //
pub const Reg_Grid_Frequency:u16        = 513; //0.01Hz
pub const Reg_Battery_Voltage_Raw:u16   	= 520;
pub const Reg_Battery_Current_Raw:u16       = 521;
pub const Reg_Battery_Power_Raw:u16         = 522;
pub const Reg_DC_Inlet_Temp_Raw:u16         = 523;
pub const Reg_AC_Active_Power_L1_Raw:u16    = 524;
pub const Reg_AC_Active_Power_L2_Raw:u16    = 525;
pub const Reg_AC_Active_Power_L3_Raw:u16    = 526;
pub const Reg_AC_Current_L1_Raw:u16         = 527;
pub const Reg_AC_Current_L2_Raw:u16         = 528;
pub const Reg_AC_Current_L3_Raw:u16         = 529;
pub const Reg_AC_Voltage_L1_Raw:u16         = 530;
pub const Reg_AC_Voltage_L2_Raw:u16         = 531;
pub const Reg_AC_Voltage_L3_Raw:u16         = 532;
pub const Reg_AC_Frequency_Raw :u16         = 533;
pub const Reg_AC_Inlet_Temp_Raw:u16         = 534;


pub const Reg_Pump_1_Command:u16        = 600;
pub const Reg_Pump_1_State:u16          = 601;
pub const Reg_Pump_1_Error:u16          = 602;
pub const Reg_Pump_1_Speed_Ref:u16      = 603;
pub const Reg_Pump_1_Speed_Actual:u16   = 604;

pub const Reg_Pump_2_Command:u16        = 620;
pub const Reg_Pump_2_State:u16          = 621;
pub const Reg_Pump_2_Error:u16          = 622;
pub const Reg_Pump_2_Speed_Ref:u16      = 623;
pub const Reg_Pump_2_Speed_Actual:u16   = 624;

pub const Reg_Estimator_Value:u16 = 700; // Maxes out at max u16 value but actual value may be larger
pub const Reg_Estimator_Hypothesis_Raw:u16 = 701; // 0 in linear region, 1 in nonlinear region
pub const Reg_Estimator_Hypothesis:u16 = 702; // 0 if linear, 1 if nonlinear + start of phase, 2 if nonlinear + end of phase 
pub const Reg_Estimator_Coulomb_Big:u16 = 713; // First half of coulomb count u32
pub const Reg_Estimator_Coulomb_Little:u16 = 714; // Second half of coulomb count u32
pub const Reg_Estimator_Coulomb_Max_Big:u16 = 715; // First half of coulomb max u32
pub const Reg_Estimator_Coulomb_Max_Little:u16 = 716; // Second half of coulomb max u32

// Registers preallocated for raw sensor values
pub const Reg_Board1_Start:u16 = 800;
pub const Reg_Board2_Start:u16 = 810;
pub const Reg_Board3_Start:u16 = 820;
pub const Reg_Board4_Start:u16 = 830;
pub const Reg_Board5_Start:u16 = 840;
pub const Reg_Board6_Start:u16 = 850;
pub const Reg_Board7_Start:u16 = 860;
pub const Reg_Board8_Start:u16 = 870;
pub const Reg_Board9_Start:u16 = 880;
pub const Reg_Board10_Start:u16 = 890;

// Registers for virtual sensors/co-simulation

pub const Reg_Sim_Temp_Anolyte_Tank:u16 = 900;
pub const Reg_Sim_Temp_Anolyte_Stack_In:u16 = 901;
pub const Reg_Sim_Temp_Anolyte_Stack_Out: u16 = 902;
pub const Reg_Sim_Temp_Catholyte_Tank:u16 = 903;
pub const Reg_Sim_Temp_Catholyte_Stack_In:u16 = 904;
pub const Reg_Sim_Temp_Catholyte_Stack_Out:u16 = 905;
pub const Reg_Sim_Pressure_Anolyte_In:u16 = 906;
pub const Reg_Sim_Pressure_Anolyte_Out:u16 = 907;
pub const Reg_Sim_Pressure_Catholyte_In:u16 = 908;
pub const Reg_Sim_Pressure_Catholyte_Out:u16 = 909;
pub const Reg_Sim_Flow_Anolyte:u16 = 910;
pub const Reg_Sim_Flow_Catholyte:u16 = 911;

pub const SIM_REG_ARRAY:[u16;12] = [
Reg_Sim_Temp_Anolyte_Tank,
Reg_Sim_Temp_Anolyte_Stack_In,
Reg_Sim_Temp_Anolyte_Stack_Out,
Reg_Sim_Temp_Catholyte_Tank,
Reg_Sim_Temp_Catholyte_Stack_In,
Reg_Sim_Temp_Catholyte_Stack_Out,
Reg_Sim_Pressure_Anolyte_In,
Reg_Sim_Pressure_Anolyte_Out,
Reg_Sim_Pressure_Catholyte_In,
Reg_Sim_Pressure_Catholyte_Out,
Reg_Sim_Flow_Anolyte,
Reg_Sim_Flow_Catholyte,
];

const NOT_APPLICABLE: f32 = 1.0;
const CUBICSEC_TO_LITERMIN : f32 = 1.0/60000.0;
const PASCAL_TO_MILLIBAR : f32 =  0.01;

// Coefficients for unit conversions: SI -> modbus units
pub const SIM_REG_ARRAY_CONVERSION: [f32;12] = [
    NOT_APPLICABLE,
    NOT_APPLICABLE,
    NOT_APPLICABLE,
    NOT_APPLICABLE,
    NOT_APPLICABLE,
    NOT_APPLICABLE,
    PASCAL_TO_MILLIBAR,
    PASCAL_TO_MILLIBAR,
    PASCAL_TO_MILLIBAR,
    PASCAL_TO_MILLIBAR,    
    CUBICSEC_TO_LITERMIN,
    CUBICSEC_TO_LITERMIN,
];

pub const Reg_TMS_Command : u16 = 1000;
pub const Reg_TMS_Anolyte_State : u16 = 1001;
pub const Reg_TMS_Anolyte_Error: u16 = 1002;
pub const Reg_TMS_Anolyte_Heating: u16 = 1003;
pub const Reg_TMS_Catholyte_State: u16 = 1004;
pub const Reg_TMS_Catholyte_Error: u16 = 1005;
pub const Reg_TMS_Catholyte_Heating: u16 = 1006;


pub const Reg_System_Gas_Risk: u16 = 1100;


pub const INVERTER_COMMAND_RESET: u16   = 0x5CAB; //resets state to idle if possible or goes to error
pub const INVERTER_COMMAND_STOP: u16 = 0; //stops charging or discharging and sets inverter idle
pub const INVERTER_COMMAND_CHARGE: u16  = 1; //starts inverter charging battery from grid
pub const INVERTER_COMMAND_DISCHARGE:u16= 2; //starts inverter discharging battery to grid

#[derive(PartialEq, PartialOrd, Clone,Debug, Copy)]
pub enum TInverterState {
    INVERTER_STATE_IDLE= 0, //inverter/ charger idle
    INVERTER_STATE_STOPPING = 10, //we start stopping the invert and waits for Trump system idle
    INVERTER_STATE_START_CHARGING = 20, //Inverter is sarting to charge from
    INVERTER_STATE_CHARGE  = 25, //inverter is in the process of charging battery from grid
    INVERTER_STATE_START_DISCHARGING= 30, //starts is  in the process of discharging battery to grid
    INVERTER_STATE_DISCHARGE= 35, //starts is  in the process of discharging battery to grid
    INVERTER_STATE_ERROR= 16384, //inverter: some error has occured
}
use crate::BMS_registers::TInverterState::*;
pub fn U16ToTInverterState(value:u16) -> TInverterState {
    const INVERTER_STATE_IDLE_u16:u16 = TInverterState::INVERTER_STATE_IDLE as u16;
    const INVERTER_STATE_STOPPING_u16:u16 = TInverterState::INVERTER_STATE_STOPPING as u16;
    const INVERTER_STATE_START_CHARGING_u16:u16 = TInverterState::INVERTER_STATE_START_CHARGING as u16;
    const INVERTER_STATE_CHARGE_u16:u16 = TInverterState::INVERTER_STATE_CHARGE as u16;
    const INVERTER_STATE_START_DISCHARGING_u16:u16 = TInverterState::INVERTER_STATE_START_DISCHARGING as u16;
    const INVERTER_STATE_DISCHARGE_u16:u16 = TInverterState::INVERTER_STATE_DISCHARGE as u16;
    const INVERTER_STATE_ERROR_u16:u16 = TInverterState::INVERTER_STATE_ERROR as u16;

    match value{
        INVERTER_STATE_IDLE_u16 => {return INVERTER_STATE_IDLE},
        INVERTER_STATE_STOPPING_u16 => {return INVERTER_STATE_STOPPING},
        INVERTER_STATE_START_CHARGING_u16 => {return INVERTER_STATE_START_CHARGING},
        INVERTER_STATE_CHARGE_u16 => {return INVERTER_STATE_CHARGE}, 
        INVERTER_STATE_START_DISCHARGING_u16=> {return INVERTER_STATE_START_DISCHARGING}, 
        INVERTER_STATE_DISCHARGE_u16=> {return INVERTER_STATE_DISCHARGE}, 
        INVERTER_STATE_ERROR_u16=> {return INVERTER_STATE_ERROR},
        _ => {return INVERTER_STATE_ERROR;}
    }
}

pub const pump_command_reset: u16   = 0x5CAB; //resets state to idle if possible or goes to error
pub const pump_command_stop: u16 = 0; //stops charging or discharging and sets inverter idle
pub const pump_command_run: u16  = 1; //starts inverter charging battery from grid

pub const pump_state_stop:u16= 0; //pump idle
pub const pump_state_running: u16  = 1; //pump is running
pub const pump_state_error:u16= 0x7fff; //pump: some error has occured

pub const SOC_BELOW_MIN:i8 = -2; //below BATTERY_SOC_MIN
pub const SOC_AT_MIN:i8 = -1; //BATTERY_SOC_MIN 
pub const SOC_BETWEEN:i8 = 0; //between BATTERY_SOC_MIN and BATTERY_SOC_MAX
pub const SOC_AT_MAX:i8 = 1; //BATTERY_SOC_MAX
pub const SOC_ABOVE_MAX:i8 = 2;  //above BATTERY_SOC_MAX
pub const SOC_UNDEFINED:i8 = 100;


#[derive(PartialEq, PartialOrd, Clone,Debug, Copy)]
pub enum TbmsState {
    BMS_STATE_STANDBY = 0, // BMS idle
    BMS_STATE_CHARGING = 1, // BMS charging battery
    BMS_STATE_DISCHARGING = 2, // BMS discharging battery
    BMS_STATE_STARTCHARGING_PUMP_WAIT = 3, // BMS initializing charge
    BMS_STATE_STARTCHARGING_TRUMPF_WAIT = 4, // BMS initializing charge
    BMS_STATE_STARTDISCHARGING_PUMP_WAIT = 5, // BMS initializing discharge
    BMS_STATE_STARTDISCHARGING_TRUMPF_WAIT = 6, // BMS initializing charge
    BMS_STATE_STOPPING = 7,
    BMS_STATE_STARTING = 8,
    BMS_STATE_GOING_TO_STANDBY = 9,
    BMS_STATE_RESET = 201,   // BMS attempting to reset due to component fault detected
    BMS_STATE_ERROR_RECOVERABLE = 400,  //This error may be cleared. 
    BMS_STATE_ERROR = 401, // BMS shutting down battery due to irrecoverable fault.
    BMS_STATE_ERROR_STOPPED = 402, //BMS has shut down all external items continues to BMS_STATE_FATAL -> main stops -> exit
    BMS_STATE_FATAL = 1000, //Fatal error detected and system *has* shut down -> exit
}

// pub const TbmsState_array:[TbmsState;14] = [
//     TbmsState::BMS_STATE_STANDBY ,
//     TbmsState::BMS_STATE_DISCHARGING ,
//     TbmsState::BMS_STATE_STARTCHARGING_PUMP_WAIT ,
//     TbmsState::BMS_STATE_STARTCHARGING_TRUMPF_WAIT ,
//     TbmsState::BMS_STATE_CHARGING ,
//     TbmsState::BMS_STATE_STARTDISCHARGING_PUMP_WAIT ,
//     TbmsState::BMS_STATE_STARTDISCHARGING_TRUMPF_WAIT ,
//     TbmsState::BMS_STATE_STOPPING ,
//     TbmsState::BMS_STATE_STARTING ,
//     TbmsState::BMS_STATE_RESET ,  
//     TbmsState::BMS_STATE_ERROR_RECOVERABLE ,
//     TbmsState::BMS_STATE_ERROR ,
//     TbmsState::BMS_STATE_ERROR_STOPPED,
//     TbmsState::BMS_STATE_FATAL ,
// ];

#[derive(PartialEq, PartialOrd, Clone,Debug, Copy)]
pub enum TErrorType {
    NO_ERROR = 0,
    RECOVERABLE  = 1,
    NON_RECOVERABLE = 2,
    FATAL = 3,
}

#[derive(PartialEq, PartialOrd, Clone,Debug, Copy)]
pub struct t_error {
    pub has_error:bool,
    pub error_type:TErrorType,
} 

pub const BMS_COMMAND_CHARGE: u16 = 1; // External command to charge
pub const BMS_COMMAND_DISCHARGE: u16 = 2; // External command to discharge
pub const BMS_COMMAND_STOP: u16 = 0; // External command to cease operation

pub const MAX_RESET_ATTEMPTS:u16 = 5;

pub type uint16 = u16;
pub type uint32 = u32;
pub type int32 = i32;
pub type int16 = i32;

// thermal parameters
const R_stack:f32 = 2.1e-04;
const R_pipes:f32 = 1.0e-03;
const R_he:f32 = 3.8e-03;
const R_ambient:f32 = 2.0*8.4e-03;
const C_stack:f32 = 4.7e03;
const C_pipes:f32 = 5.2e04;
const C_he:f32 = 4.7e05;

// hydraulic parameters
const viscosity:f32 = 0.006; // Electrolyte viscosity
const permeability:f32 = 1.685e-10; // Electrode permeability
const density:f32 = 1400.0;
const length:f32 = 0.26; // Half-cell length
const width:f32 = 0.30; // Half-cell width
const depth:f32 = 3e-03; // Half-cell depth
const Nseries:f32 = 15.0; // Number of stack cells in series

const P_1atm:f32 =1013.25; //hPA (1 hektoPascal = 100 Pascal) 
const C_water:f32 = 4184.0; //J/(kg*K)

const MIN_DISCHARGE_VOLTAGE:f32 = 5.0; //do not discharge below this voltage level.

// logging.basicConfig()
// log = logging.getLogger()
// log.setLevel(logging.ERROR)

// --------------------------------------------------------------------------- //
// Precalculations for battery and energy
// --------------------------------------------------------------------------- //
// const kilo:f32 = 1e3;
// const mega:f32 = 1e6;
// const giga:f32 = 1e9;
// const milli:f32 = 1e-3;
// const micro:f32 = 1e-6;
// const nano:f32 = 1e-9;
// const seconds_per_hour:f32 = 60.0*60.0;
// const Wh_to_J:f32 = seconds_per_hour;
// const kWh_to_J:f32 = kilo * seconds_per_hour;
// const J_to_kWh:f32 = 1.0/kWh_to_J;
// const J_to_Wh:f32 = 1.0/seconds_per_hour;

// // for vanadium:
// //// Specific energy	10–20 Wh/kg (36–72 J/g)
// //// Energy density	15–25 Wh/L (54–65 kJ/L)
// //// Charge/discharge efficiency	75–80%<.[1][2]
// //// Time durability	20-30 years
// //// Cycle durability	>12,000-14,000 cycles[3]
// ////Nominal cell voltage	1.15–1.55 V
// const e_cufb:f32 = 7.0*Wh_to_J; //Wh/L  J*s/L
// const E_tank:f32 = 25.0*kWh_to_J; //kWh -> J
// const tanksize:f32 = E_tank / e_cufb;
// // print('Tanksize =', tanksize,' liter')
// // print('Energy density = ', e_cufb,' [J/L]')
// // print('Energy in tank = ', E_tank,' [J]')
// const flow_per_Watt:f32 = 1.0 / e_cufb;
// // print('Flow in liter per joule = ', flow_per_Watt,' [l]')


pub fn int(value:f32)-> uint16{
	return value as uint16;
}

pub fn float(value:uint16) -> f32 {
	return value as f32;
}

pub struct BMS_Handler {
    holding_registers: Vec<u16>,
}

pub fn handle_signed_int(value:u16) -> i16{
    //twos complement
    if value >= 0x8000 { 
        let mut tempvalue = value ^ 0xFFFF; //xor flip
        tempvalue += 1;
        return (tempvalue as i16) * (-1 as i16); 
    } else {
        return (value & 0x7FFF) as i16;
    }
}

impl BMS_Handler {
    pub fn new(holding_registers: Vec<u16>) -> Self {
        Self { holding_registers }
    }

    pub fn holding_registers_as_mut(&mut self) -> &mut [u16] {
        self.holding_registers.as_mut_slice()
    }

}

pub fn read_register_mutex(
    handler: &ServerHandlerType<BMS_Handler>, 
    register_to_read: uint16) -> u16 {
    let mut handler_mutex = handler.lock().unwrap();
    let readval =  handler_mutex.holding_registers_as_mut()[register_to_read as usize] as u16;
    drop(handler_mutex);
    return readval;
}

pub fn write_register_mutex(
    handler: &ServerHandlerType<BMS_Handler>, 
    register_to_write: u16,
    value_to_write: u16){
    let mut handler_mutex = handler.lock().unwrap();
    handler_mutex.holding_registers_as_mut()[register_to_write as usize] = value_to_write;
    drop(handler_mutex);
}

impl RequestHandler for BMS_Handler {
    fn read_holding_register(&self, address: u16) -> Result<u16, ExceptionCode> {
        // println!("registers read {}", address);
        print!("<read {:?} = {:?}> ", address, self.holding_registers.get(address as usize).to_result());
        self.holding_registers.get(address as usize).to_result()
    }

    fn write_single_register(&mut self, value: Indexed<u16>) -> Result<(), ExceptionCode> {
        // tracing::info!(
        //     "write single register, index: {} value: {}",
        //     value.index,
        //     value.value
        // );

        if let Some(reg) = self.holding_registers.get_mut(value.index as usize) {
            *reg = value.value;
            Ok(())
        } else {
            Err(ExceptionCode::IllegalDataAddress)
        }
    }

    fn write_multiple_registers(&mut self, values: WriteRegisters) -> Result<(), ExceptionCode> {
        // tracing::info!("write multiple registers {:?}", values.range);

        let mut result = Ok(());

        for value in values.iterator {
            if let Some(reg) = self.holding_registers.get_mut(value.index as usize) {
                *reg = value.value;
            } else {
                result = Err(ExceptionCode::IllegalDataAddress)
            }
        }

        result
    }
}

pub fn create_handler() -> (
    ServerHandlerType<BMS_Handler>,
    ServerHandlerMap<BMS_Handler>,
) {
    // ANCHOR: handler_map_create
    let handler = BMS_Handler::new(vec![0; 4500]).wrap();

    // map unit ids to a handler for processing requests
    let map = ServerHandlerMap::single(UnitId::new(1), handler.clone());
    // ANCHOR_END: handler_map_create

    (handler, map)
}
