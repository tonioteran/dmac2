# dmac2

ROS2 driver for the Evologics Underwater Modems

## Basic commands

 - `ATC` - command mode
 - `ATO` - data mode
 - `AT@ZX` - extended notifications {0:off 1:on}
 - `AT?RP` - promiscuous mode {0:off 1:on}
 - `AT@ZU` - show USBL messages {0:off 1:on}
 - `ATZ` - restart
 - `ATI` - 0:firmware 1:phy_proto,data_prot 7:manufacturer
 - `AT&W` - store settings
 - `AT?L` - source level (from highest to lowest): {0,1,2,3}
 - `AT?C` - waveform (2 for network)
 - `AT?AL` - local address
 - `AT?CA` - sound speed
 - `AT?T` - time of flight (ms)
 - `AT?AM` - maximum number of nodes {2,6,14,30,62,126,254}
 - `ATN` - noise mode
 - `AT?E` - noise profiling

 - Instant messages: 
```
AT*SENDIM,<length>,<destination address>,<flag>,<data>`
DELIVEREDIM,<destination address>`
FAILEDIM,<destination address>`
CANCELLEDIM,<destination address>`
RECVIM,<length>,<source address>,<destination address>,<flag>,
       <duration>,<rssi>,<integrity>,<velocity>,<data>
```
 - Piggyback messages:
```
AT*SENDPBM,<length>,<destination address>,<data>
CANCELLEDPBM,<destination address>
RECVPBM,<length>,<source address>,<destination address>,
        <duration>,<rssi>,<integrity>,<velocity>,<data>
```
 - Position measurements
```
USBLLONG,<current time>,<measurement time>,<remote address>,
<X>,<Y>,<Z>,<E>,<N>,<U>,<roll>,<pitch>,<yaw>,<propagation time>,
<rssi>,<integrity>,<accuracy>
USBLANGLES,<current time>,<measurement time>,<remote_address>,<lbearing>,
<lelevation>,<bearing>,<elevation>,<roll,>,<pitch>,<yaw>,<rssi>,<integrity>,
<accuracy>
```

## Other commands
 - `ATH` - close connection
 - `AT?S` - connection status
 - `AT&V` - device settings
 - `AT?LC` - source level control
 - `AT?G` - gain {0 = normal, 1 = low (-20 dB)}
 - `AT?AR` - remote address
 - `AT?ZC` - cluster size
 - `AT?ZP` - packet time
 - `AT?RC` - retry count
 - `AT?RT` - retry timeout
 - `AT?ZI` - idle timeout
 - `AT?ZS` - stream number
 - `AT?ZSL` - stream number list
 - `AT?ZL` - pool size
 - `AT?ZD` - propagation counter
 - `AT?ZO` - overflow counter
 - `AT?UT` - system time
 - `AT?RI` - IM retry count
 - `AT?DI` - IM delivery status
 - `AT?DA` - wakeup active time
 - `AT?DT` - wakeup period
 - `AT?ZH` - hold timeout
 - `AT?BL` - local to remote bitrate
 - `AT?BR` - remote to local bitrate
 - `AT?E` - rssi (dB respect 1V)
 - `AT?I` - signal integrity
 - `AT?V` - relative velocity
 - `AT?P` - multipath structure
 - `AT&F` - reset factory settings
 - `AT?PID` - Protocol ID
 - `ATN -` noise mode
 - `AT&E` - signal strength measurement
 - `AT?NOISE` - noise analysis `{NOISE,<size>,<sample rate>,<gain>,<rssi>,<data>}`
 - `ATA -` back to listen state
 - `ATS -` switch to deaf state
 - `AT?UP` - estimate pos `<sec>,<addr>,<x>,<y>,<z>`
 - `AT?UPX` - estimate compensated pose `<sec>,<addr>,<E>,<N>,<U>`

## End of line:
 - Ethernet: `\n`
 - Serial: `\r` 
 - Answers: `\r\n` 

## Dev Notes

TODO: delete this once the port is over.

- For the new logging in ROS2, this was useful: https://robotics.stackexchange.com/questions/96379/global-logger-for-logging-without-a-node.
- Jun 13 11:45 testing, successful connection to LoLo's top-side unit. Sample output:

```
niklas@niklas-lolo-laptop:~/ros2_ws$ ros2 run dmac2 dmac                      
[INFO] [1749807068.046989751] [dmac2_logger]: Spinning...                        
[INFO] [1749807068.047262856] [dmac2_logger]: Connecting to IP: 192.168.0.206, port: 9200
[INFO] [1749807068.066478748] [dmac2_logger]: hasAHRS: 0      
[INFO] [1749807068.066564580] [dmac2_logger]: Adding the parser node for TCP/IP mode
[WARN] [1749807068.066874552] [dmac2_logger]: do_close by handle_connect          
[WARN] [1749807068.066947899] [dmac2_logger]: connection closed...
[INFO] [1749807068.066997505] [dmac2_logger]: ini: disconnected
[INFO] [1749807069.067296499] [dmac2_logger]: reconnecting
[WARN] [1749807069.067549206] [dmac2_logger]: do_close by handle_connect         
[WARN] [1749807069.067621084] [dmac2_logger]: connection closed...            
[INFO] [1749807069.067716634] [dmac2_logger]: ini: disconnected                  
[INFO] [1749807070.067977618] [dmac2_logger]: reconnecting                    
[WARN] [1749807070.068229130] [dmac2_logger]: do_close by handle_connect     
[WARN] [1749807070.068297209] [dmac2_logger]: connection closed...        
[INFO] [1749807070.068394903] [dmac2_logger]: ini: disconnected   
^C[INFO] [1749807070.236243727] [rclcpp]: signal_handler(signum=2)
niklas@niklas-lolo-laptop:~/ros2_ws$ ros2 run dmac2 dmac
[INFO] [1749807148.059284017] [dmac2_logger]: Spinning...                
[INFO] [1749807148.059491063] [dmac2_logger]: Connecting to IP: 192.168.0.206, port: 9200
[INFO] [1749807148.078917385] [dmac2_logger]: hasAHRS: 0
[INFO] [1749807148.079093278] [dmac2_logger]: Adding the parser node for TCP/IP mode
[INFO] [1749807148.080103887] [dmac2_logger]: ini: connected
[INFO] [1749807148.080138499] [dmac2_logger]: (IDLE) -> INTERNAL -> (IDLE)
[INFO] [1749807148.080152362] [dmac2_logger]: Handling INTERNAL in state IDLE
[INFO] [1749807148.080168045] [dmac2_logger]: sending +++AT?MODE

[INFO] [1749807148.084507818] [dmac2_logger]: Parsing new data(0): +++AT?MODE:2:AT

[INFO] [1749807148.084529468] [dmac2_logger]: waitsync_ : 1
[INFO] [1749807148.084538188] [dmac2_logger]: more_:  
[INFO] [1749807148.093500542] [dmac2_logger]: (IDLE) -> RCV -> (HANDLE_MODE)
[INFO] [1749807148.093535372] [dmac2_logger]: Handling RCV in state HANDLE_MODE
[INFO] [1749807148.093564472] [dmac2_logger]: command ?MODE, report: AT

[INFO] [1749807148.093581452] [dmac2_logger]: sending +++ATO

[INFO] [1749807148.093707409] [dmac2_logger]: (HANDLE_MODE) -> YAR -> (HANDLE_YAR)
[INFO] [1749807148.093723302] [dmac2_logger]: Handling YAR in state HANDLE_YAR
[INFO] [1749807148.093737795] [dmac2_logger]: sending +++AT@CTRL
```




