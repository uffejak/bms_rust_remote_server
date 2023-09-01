#![allow(warnings)]

extern crate rust_test as BMS;

use mt_logger::*;
use BMS::BMS_registers::*;

use std::error::Error;
use std::process::exit;
use std::{thread, time};
use tokio::runtime::*;

use rodbus::*;
use rodbus::server::*;
//use std::sync::mpsc::Sender;
//use std::sync::mpsc::Receiver;

use std::sync::mpsc;

use std::process::Command;

pub const TICKRATE_REMOTESERVER: u64 = 1;
pub const PORT_REMOTESERVER: &str = "127.0.0.1:5021";

// ANCHOR: runtime_init
#[tokio::main(flavor = "multi_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    mt_new!(Some("RemoteServer_V0.0.1_"), Level::Trace, OutputStream::File);
    mt_log!(Level::Info, "Remote server booting");

    // Define memory shared by threads (holding registers for BMS)
    let (handler_remoteserver, map_remoteserver) = create_handler();

    // Define all the communication channels between program threads (unused at this time)
    //let (tx_void, rx_void): ( mpsc::Sender<ChannelStruct>, mpsc::Receiver<ChannelStruct>) = mpsc::channel();
    
    // ANCHOR: logging
    tracing_subscriber::fmt()
        .with_max_level(tracing::Level::ERROR)
        .with_target(false)
        .init();

    // Define process tick rates
    let tickrate_remoteserver: u64 = TICKRATE_REMOTESERVER;

    let server_address = PORT_REMOTESERVER;

    // Set up runtimes for each thread
    let server = rodbus::server::spawn_tcp_server_task(
        2,
        server_address.parse()?,
        map_remoteserver,
        AddressFilter::Any,
        DecodeLevel::default(),
    ).await?;

    let mut interval = tokio::time::interval(std::time::Duration::from_secs(TICKRATE_REMOTESERVER));

    loop {
        interval.tick().await;
    }
}
