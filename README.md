# IMU
Code is in Scilab

 ### output
 <ul>
  <li>  eangle  ...euler  angle </li>
   <li> gryo  </li>
   <li> acc  </li>
   <li> mag </li>
  </ul>
  

###  Input

 <ul>
  <li>  fname ..  9 axis sensor samples collected from PPN via UART  </li>
   <li>  removed in current version ...rowval ... noumber of readings ..need to be processed </li>
  </ul>
      
      
  ##  function [eangle, gyro, acc, magV] = imuval(fname)
  

## function FilterSensorData(fname)
 - ////  filter 9 axis data by using IIR filter
 - //    function FilterSensorData(fname,rowval)
