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
   <li> rowval ... noumber of readings ..need to be processed </li>
  </ul>
      
      
    ##  function [eangle, gyro, acc, magV] = imuval(fname)
