//////////////////////////////////////////////
//  IMU Project ...9 Axis sensor
///  jk
///  9-Jan-2018
//
//   http://x-io.co.uk/gait-tracking-with-x-imu/
////////////////////////////////////////////////
// output ...
//   eangle  ...euler  angle
//   gryo
//   acc
//   mag
//
//  Input
//      fname ..  9 axis sensor samples collected from PPN via UART
//      rowval ... noumber of readings ..need to be processed
//function [eangle, gyro, acc, magV] = imuval(fname,rowval)
function [eangle, gyro, acc, magV] = imuval(fname)
 mclose('all');
  
 f1 = mopen(fname,"r");
 //v1= mgetl(f1,rowval);
 v1= mgetl(f1);
 b=size(v1);
 NumberOfSamples = b(1);
 sensorType =0;
 comsense =[];
 gyrosense =[];
 accsense = [];
 magsense =[];

 
 //for row=1:rowval   
 for row=1:NumberOfSamples   
   a1= v1(row);
  // disp(row);
   //disp(a1);
   b1=ascii(a1);
   x=size(b1);
   len = x(2);
   sgnval = 1;
   wcount=1;
   senseRow =[];
   r1=[];
   o32=0;
   
   for i=1:len
       
       val = b1(i);
       // str1 = sprintf( "%d %d",i,val);
       // disp(str1);
       if( val  ~= 32)
         if ( val == 45)
             sgnval =-1;
         else
             r1 = [r1 val];
             //disp(r1);
            // str2 =sprintf( "up wordcount %d",wcount);
            // disp(str2); 
         end
         o32=0;
       else
           if (o32 == 0)
              
               val2 =  ascii(r1);
            
              if ( wcount == 1)
                 //disp(val2) ;
                 if (  ( val2 =='[[COM') || (val2 =='[COM') )
                     sensorType =1;
                 end
                 
                 if (   val2 =='[[GYR' )
                     sensorType =2;
                 end
                 
                  if (   val2 =='[[ACC' )
                     sensorType =3;
                  end
                  
                  if (   val2 =='[[MAG' )
                     sensorType =4;
                  end
                 
                 
              end
              
               if ( wcount > 1)
                  val3= strtod(val2);
                  val3 = val3*sgnval;
                 // disp(val3);
                   senseRow =[senseRow val3];
                 
               end
              
               wcount = wcount+1;
               r1 =[];
               sgnval =1;
               o32=1;
               
           else
               o32 = o32+1;
           end
           
       end
       if (  (wcount == 6) & ( i==len) )
           
          if ( sensorType ~=3)
           val2 =  ascii(r1); 
           val3= strtod(val2);
           val3 = val3*sgnval;
           //disp(val3);
           senseRow =[senseRow val3];
          end
       end
     end  // i for loop
   
    if ( sensorType ==1) 
         comsense =[comsense; senseRow];
    end
                  
     if ( sensorType ==2) 
         gyrosense =[gyrosense ; senseRow];
     end
                  
     if ( sensorType ==3) 
        accsense =[accsense ; senseRow];
     end
                  
     if ( sensorType ==4) 
        magsense =[magsense ; senseRow];
     end
   sensorType =0;
end

  eangle =comsense;
  gyro = gyrosense;
  acc = accsense;
  magV = magsense ;
  
  dispval =0;
  if (dispval == 1) 
    subplot(221), plot( eangle(:,2:4)); title("COM");
    subplot(222), plot(gyro(:,2:4)); title("GYRO");
    subplot(223), plot(acc(:,2:4)); title("Accleration");
    subplot(224), plot(mag(:,2:4)); title("Mag");
  end;  // pause();

endfunction

////////////////////////////////
//  Input  .. Fs  is sampling Frequency 
//  Output  ...  s1 in state transfer.. iir filter coeff
//
//  ref study [hs,pols,zers,gain]=analpf(n,fdesign,rp,omega)
//       analog low pass filter
function  [sl]=jFilt(Fs)
Order   = 12; // 2; // The order of the filter
//Fs      = 1000; // The sampling frequency
Fcutoff = 30;   // The cutoff frequency

// We design a low pass elliptic filter
hz = iir(Order,'lp','ellip',[Fcutoff/Fs/2 0],[0.1 0.1]);

// We compute the frequency response of the filter
[frq,repf]=repfreq(hz,0:0.001:0.5);
[db_repf, phi_repf] = dbphi(repf);

// And plot the bode like representation of the digital filter
 dispval =0;
  if (dispval == 1) 
   subplot(2,1,1);
   plot2d(Fs*frq,db_repf);
   xtitle('Obtained Frequency Response (Magnitude)');
   subplot(2,1,2);
   plot2d(Fs*frq,phi_repf);
   xtitle('Obtained Frequency Response (Phase in degree)');
  end
 
sl= tf2ss(hz); // From transfer function to syslin representation

endfunction
 
// We design a high pass elliptic filter
function  [sl]=jhFilt(Fs)
  Order   = 2; // 2; // The order of the filter
  Fcutoff = 0.001;   // The cutoff frequency
  hz = iir(Order,'hp','ellip',[Fcutoff/Fs/2 0],[0.1 0.1]);
  sl= tf2ss(hz); // From transfer function to syslin representation

endfunction

//return angle , unit: 1/10 degree
// int16_t _atan2(int32_t y, int32_t x){
function [a]=angleCompute(ay,ax)
  az =ay;
  c = abs(ay) < abs(ax);
  if ( c ) 
      az = az / ax; 
  else 
      az = ax / az;
  end
  
  aval = 2046.43 * (az / (3.5714 +  (az * az)));
  if ( c )
   if (ax<0) 
     if (ay  < 0 )
         aval = ( aval - 1800);
        //  a = val;
     else
          aval = aval + 1800;
     end
   end
  else 
    aval = 900 - aval;
    if (ay<0) 
       // a -= 1800;
       aval = aval - 1800;
    end
  end
  a = aval;
  //return a;
endfunction
 
 
 
function ab = quaternProd(a, b)
            ab(:,1) = a(:,1).*b(:,1)-a(:,2).*b(:,2)-a(:,3).*b(:,3)-a(:,4).*b(:,4);
            ab(:,2) = a(:,1).*b(:,2)+a(:,2).*b(:,1)+a(:,3).*b(:,4)-a(:,4).*b(:,3);
            ab(:,3) = a(:,1).*b(:,3)-a(:,2).*b(:,4)+a(:,3).*b(:,1)+a(:,4).*b(:,2);
            ab(:,4) = a(:,1).*b(:,4)+a(:,2).*b(:,3)-a(:,3).*b(:,2)+a(:,4).*b(:,1);
endfunction
  
function qConj = quaternConj(q)
          qConj = [q(:,1) -q(:,2) -q(:,3) -q(:,4)];
endfunction
   
function obj = Quaternion(obj, value)
            if(norm(value) == 0)
                error('Quaternion magnitude cannot be zero.');
            end
            value = value / norm(value);
            obj.Quaternion = value;
            obj.q = obj.quaternConj(value);
endfunction       

function obj = Reset(obj)
            obj.KpRamped = obj.KpInit;   //   % start Kp ramp-down
            obj.IntError = [0 0 0]';    //  	% reset integral terms
            obj.q = [1 0 0 0];       //    	% set quaternion to alignment	
endfunction  
 
function obj = UpdateIMU(obj, Gyroscope, Accelerometer)
//            % Normalise accelerometer measurement
            if(norm(Accelerometer) == 0) //                 	% handle NaN
                disp('Accelerometer magnitude is zero.  Algorithm update aborted.');
            else
                Accelerometer = Accelerometer / norm(Accelerometer); //   % normalise measurement
             // Compute error between estimated and measured direction of gravity
               v = [2*(obj.q(2)*obj.q(4) - obj.q(1)*obj.q(3))
                    2*(obj.q(1)*obj.q(2) + obj.q(3)*obj.q(4))
                    obj.q(1)^2 - obj.q(2)^2 - obj.q(3)^2 + obj.q(4)^2]; //% estimated direction of gravity
               error1 = cross(v, Accelerometer');
               obj.IntError = obj.IntError + error1;   
               // compute integral feedback terms (only outside of init period)
               // Apply feedback terms
               Ref = Gyroscope - (obj.Kp*error1 + obj.Ki*obj.IntError)';
               // Compute rate of change of quaternion
               pDot = 0.5 * quaternProd(obj.q, [0 Ref(1) Ref(2) Ref(3)]);  
                    // compute rate of change of quaternion
               obj.q = obj.q + (pDot * obj.SamplePeriod); //  % integrate rate of change of quaternion
               obj.q = obj.q / norm(obj.q); //   % normalise quaternion
               // Store conjugate
               obj.Quaternion = quaternConj(obj.q);
            end
endfunction
 
//function obj = AHRSset(varargin)          
function obj = AHRSset(obj,Kp,Ki,KpInit,InitPeriod,SamplePeriod,Quaternion)
                obj.Quaternion = Quaternion;
                obj.q = quaternConj(obj.Quaternion);
                obj.Kp = Kp;
                obj.Ki = Ki;
                obj.KpInit = KpInit;
                obj.InitPeriod = InitPeriod;  
                obj.SamplePeriod = SamplePeriod;                 
                obj.KpRamped = KpInit;
endfunction
                   
function [AHRS]= GetAHRS(Fs)
        AHRS.SamplePeriod = 1/Fs;
        AHRS.Quaternion = [1 0 0 0]; 
          //output quaternion describing the sensor relative to the Earth
        AHRS.Kp = 2;   //                  % proportional gain
        AHRS.Ki = 0;   //                  % integral gain
        AHRS.KpInit = 200;  // proportional gain used during initialisation
        AHRS.InitPeriod = 5; // initialisation period in seconds
        AHRS.q = quaternConj(AHRS.Quaternion) ; //[1 0 0 0]; // internal quaternion describing
                            // the Earth relative to the sensor
        AHRS.IntError = [0 0 0]';  //      % integral error
        AHRS.KpRamped=200;   //  % internal proportional gain 
                    // used to ramp during initialisation
    //function obj = AHRSset(obj,Kp,Ki,KpInit,InitPeriod,SamplePeriod,Quaternion)
             
       // quat = zeros(length(timeV), 4);
             
endfunction
////  filter 9 axis data by using IIR filter
 
//function FilterSensorData(fname,rowval)
function FilterSensorData(fname)
     Fs = 120;  // sampling rate..
     SampleTime = 1/Fs;
                // .defined in 9 axis example of PPN
     
     sl=jFilt(Fs);
     sh=jhFilt(Fs);
    // [eangle, gyro, acc, magV] = imuval(fname,rowval);
     [eangle, gyro, acc, magV] = imuval(fname);
     acc = acc*(1/1000);  // device scale up b4 UART Push
     
     //disp( size(gyro))
     
    // xval = 1:rowval ; //  x axis points
     xGyro = gyro(:,2);
     yGyro = gyro(:,3);
     zGyro = gyro(:,4);
     
     
 
     
    
      xGyro =xGyro';
      xvallen = length(xGyro); //  x axis points
      xval = [];
      sum1=0;
      for i=1:xvallen
          sum1 = sum1 + SampleTime;
          xval = [xval sum1];
      end
      oxGyro = flts(xGyro,sl); // Filter Gyro sensor signal
      subplot(2,1,1);
      plot(xval,xGyro);
      xtitle(' Gyro X sensor data ','Sample Time','x gyro');
      subplot(2,1,2);
      plot(xval,oxGyro);
      xtitle('Filtered X Gyro','Sample Time','oxGyro');
      xclick();
      clf();

      
      yGyro =yGyro';
      //xval = 1:length(yGyro); //  x axis points
      oyGyro = flts(yGyro,sl); // Filter Gyro sensor signal
      subplot(2,1,1);
      plot(xval,yGyro);
      xtitle('  Y Gyro sensor data ','Sample Time','yGyro');
      subplot(2,1,2);
      plot(xval,oyGyro);
      xtitle('Filtered Y Gyro','Sample Time','oyGyro');
      xclick();
      clf();

      
      zGyro =zGyro';
      //xval = 1:length(zGyro); //  x axis points
      ozGyro = flts(zGyro,sl); // Filter Gyro sensor signal
      subplot(2,1,1);
      plot(xval,zGyro);
      xtitle(' zGyro  Z sensor data ','Sample Time','zGyro');
      subplot(2,1,2);
      plot(xval,ozGyro);
      xtitle('Filtered Z Gyro','Sample Time','ozGyro');
      xclick();
      clf();
      
      
      
      xAcc = acc(:,2);
      yAcc = acc(:,3);
      zAcc = acc(:,4);
      xAcc =xAcc';
      yAcc =yAcc';
      zAcc =zAcc';
      
     
      
      
      
       
       xvallen = length(xAcc); //  x axis points
       xval = [];
       sum1=0;
       for i=1:xvallen
          sum1 = sum1 + SampleTime;
          xval = [xval sum1];
       end
       
       accMag = sqrt(xAcc.*xAcc + yAcc.*yAcc + zAcc.*zAcc);
       accMagFilt = flts(accMag,sh);  // high pass filter
       accMagFiltMag = abs(accMagFilt);
       accMagv1 = flts(accMagFiltMag,sl);  // low pass filter
       // Threshold detection
       // stationary = accMagv1 < 0.05;
       stationary = [];
       xLen1= length(accMagv1);
       val = 0;
       Thold = 9.5 ; //500; //0.05 threshold
       for i=1:xLen1
          sval =accMagv1(i);
          if ( sval <  Thold)
            stationary = [stationary val];
          else
            stationary = [stationary sval];
          end
       end
       
       
       //hold on;
       set(gca(),"auto_clear","off")
       plot(xval,xAcc,'r');
       plot(xval,yAcc,'g');
       plot(xval,zAcc,'b');
       xtitle(' Accelerometer','Time (s)','Acceleration (g)');
       // hold off;
       xclick();
       set(gca(),"auto_clear","on")
       clf();
       
       
         //hold on;
       set(gca(),"auto_clear","off")
       plot(xval,accMagv1,'m:');
       plot(xval,stationary,'k:');
       xtitle(' stationary','Time (s)','Acceleration (g)');
       // hold off;
       xclick();
       set(gca(),"auto_clear","on")
       clf();
         
        quat = zeros( xvallen, 4);
        AHRS= GetAHRS(Fs);
        //UpdateIMU
        
        // For all data
          for i = 1:xvallen
            if(stationary(i))
              AHRS.Kp = 0.5;
            else
              AHRS.Kp = 0;
            end
            gyroVal = [xGyro(i) yGyro(i) zGyro(i)];
            accVal = [xAcc(i) yAcc(i) zAcc(i)];
            RScale = %pi*(1/180);  // degree to radian
           // Rad1 =deg2rad(gyroVal);
            Radval =[xGyro(i)*RScale yGyro(i)*RScale zGyro(i)*RScale];
            AHRS=UpdateIMU(AHRS,Radval,accVal);
            quat(i,:) = AHRS.Quaternion;
          end
       
          // Compute translational accelerations
          // Rotate body accelerations to Earth frame
           oxAcc = flts(xAcc,sl); // Filter accleration x-axis sensor signal
           oyAcc = flts(yAcc,sl); // Filter accleration y axis sensor signal
           ozAcc = flts(zAcc,sl); // Filter accleration z axis sensor signal
           
           acc = quaternRotate([oxAcc oyAcc ozAcc], quaternConj(quat));
           // % Convert acceleration measurements to m/s/s
           acc = acc * 9.81;
           xvalT = xval';
            set(gca(),"auto_clear","off")
            plot(xvalT,acc(:,1),'r');
            plot(xvalT,acc(:,2),'g');
            plot(xvalT,acc(:,3),'b');
            xtitle(' Accelerometer','Time (s)','Acc m/s/s ');
            // hold off;
            xclick();
            set(gca(),"auto_clear","on")
            clf();
            
            // Compute translational velocities
            acc(:,3) = acc(:,3) - 9.81;
            // Integrate acceleration to yield velocity
            velold = [0 0 0];
            vel(1,:) =  velold;
            for t = 2:xvallen
              vel(t,:) = velold + acc(t,:) * SampleTime;
              velold = vel(t,:);
              if(stationary(t) == 0)
                 vel(t,:) = [0 0 0];// force zero velocity when foot stationary
              end
            end
            
            
            // Compute integral drift during non-stationary periods
            //velDrift = zeros(size(vel));
            
            // Remove integral drift
            //vel = vel - velDrift;
            
           
            
            set(gca(),"auto_clear","off")
            plot(xvalT,vel(:,1),'r');
            plot(xvalT,vel(:,2),'g');
            plot(xvalT,vel(:,3),'b');
            xtitle(' Velocity','Time (s)','Position m / s');
            xclick();
            set(gca(),"auto_clear","on")
            clf();
            
            
            // Compute translational position

            //% Integrate velocity to yield position
            posOld = [0 0 0];
            pos(1,:)= posOld;
            for t = 2:xvallen
              pos(t,:) =  posOld  + vel(t,:) * SampleTime;  
              posOld  = pos(t,:)
              //% integrate velocity to yield position
            end

            set(gca(),"auto_clear","off")
            plot(xvalT,pos(:,1),'r');
            plot(xvalT,pos(:,2),'g');
            plot(xvalT,pos(:,3),'b');
            xtitle(' position','Time (s)','Position m ');
            // hold off;
            xclick();
            set(gca(),"auto_clear","on")
            clf();
            
            plot3d(pos(:,1),pos(:,2),pos(:,3));
            xclick();
            clf();
       
      
       
       subplot(2,2,1);
       plot(xval,xAcc);
       xtitle(' Accletation  X sensor data ','Sample Time','xAcc');
       subplot(2,2,2);
       plot(xval,oxAcc);
       xtitle('Filtered X Acc','Sample Time','oxAcc');
       //xclick();
       //clf();
       
       len1 = length(oxAcc);
       deltaT = 1/120;
       t1=[];
       val =0;
       v1x =[];
       d1x =[];
       v1y =[];
       d1y =[];
       v1z =[];
       d1z =[];
       for i=1:len1
           val = val + deltaT;
           t1 = [t1 val];
           val2 =inttrap(t1(1:i),oxAcc(1:i)); // velocity
           v1x =[v1x val2];
           val2 =inttrap(t1(1:i),oyAcc(1:i)); // velocity
           v1y =[v1y val2]
           val2 =inttrap(t1(1:i),ozAcc(1:i)); // velocity
           v1z =[v1z val2]
       end
       len2 = length(v1x);
       t2= t1(1:len2);
       for i=1:len2
           val3 =inttrap(t2(1:i),v1x(1:i)); // distance
           d1x =[d1x val3];
           val3 =inttrap(t2(1:i),v1y(1:i)); // distance
           d1y =[d1y val3];
           val3 =inttrap(t2(1:i),v1z(1:i)); // distance
           d1z =[d1z val3];
       end
      // disp(size(t2));
      // disp(size(d1x));
       len3 = length(d1x);
       t3= t1(1:len3);
       subplot(2,2,3);
       plot(t2,v1x);
       xtitle(' Velocity in x axis','time','v1x');
       subplot(2,2,4);
       plot(t3,d1x);
       xtitle('distance in X axis','time','d1x');
       xclick();
       clf();
       
       
      
 
 

       
      
       subplot(2,2,1);
       plot(xval,yAcc);
       xtitle(' Accletation  Y sensor data ','Sample Time','yAcc');
       subplot(2,2,2);
       plot(xval,oyAcc);
       xtitle('Filtered Y Acc','Sample Time','oyAcc');
       //xclick();
       //clf();
       subplot(2,2,3);
       plot(t2,v1y);
       xtitle(' Velocity in y axis','time','v1y');
       subplot(2,2,4);
       plot(t3,d1y);
       xtitle('distance in Y axis','time','d1y');
       xclick();
       clf();
       
       
      
       subplot(2,2,1);
       plot(xval,zAcc);
       xtitle(' Accletation  Z sensor data ','Sample Time','zAcc');
       subplot(2,2,2);
       plot(xval,ozAcc);
       xtitle('Filtered Z Acc','Sample Time','ozAcc');
       //xclick();
       //clf();
       
       subplot(2,2,3);
       plot(t2,v1z);
       xtitle(' Velocity in z axis','time','v1z');
       subplot(2,2,4);
       plot(t3,d1z);
       xtitle('distance in Z axis','time','d1z');
       xclick();
       clf();
       
     
     
          
       xMag = magV(:,2);
       yMag = magV(:,3);
       zMag = magV(:,4);
     
       xMag =xMag';
       xvallen = length(xMag); //  x axis points
       xval = [];
       sum1=0;
       for i=1:xvallen
          sum1 = sum1 + SampleTime;
          xval = [xval sum1];
       end
      
       oxMag = flts(xMag,sl); // Filter magnetic sensor signal
       subplot(2,1,1);
       plot(xval,xMag);
       xtitle(' Magnetic  X sensor data ','Sample Time','x Magnetic');
       subplot(2,1,2);
       plot(xval,oxMag);
       xtitle('Filtered Magnetic  X sensor data','Sample Time','oxMag');
       xclick();
       clf();
       
       yMag =yMag';
       xval = 1:length(yMag); //  x axis points
       oyMag = flts(yMag,sl); // Filter magnetic sensor signal
       subplot(2,1,1);
       plot(xval,yMag);
       xtitle(' Magnetic  Y sensor data ','Sample Time','y Magnetic');
       subplot(2,1,2);
       plot(xval,oyMag);
       xtitle('Filtered Magnetic  Y sensor data','Sample Time','oyMag');
       xclick();
       clf();
       
       zMag =zMag';
       xval = 1:length(zMag); //  x axis points
       ozMag = flts(zMag,sl); // Filter magnetic sensor signal
       subplot(2,1,1);
       plot(xval,zMag);
       xtitle(' Magnetic  Z sensor data ','Sample Time','z Magnetic');
       subplot(2,1,2);
       plot(xval,ozMag);
       xtitle('Filtered Magnetic  Z sensor data','Sample Time','ozMag');
       xclick();
       //clf();
       
       

 endfunction
 

function R = quatern2rotMat(q)
    [rows cols] = size(q);
    R = zeros(3,3, rows);
    R(1,1,:) = 2.*q(:,1).^2-1+2.*q(:,2).^2;
    R(1,2,:) = 2.*(q(:,2).*q(:,3)+q(:,1).*q(:,4));
    R(1,3,:) = 2.*(q(:,2).*q(:,4)-q(:,1).*q(:,3));
    R(2,1,:) = 2.*(q(:,2).*q(:,3)-q(:,1).*q(:,4));
    R(2,2,:) = 2.*q(:,1).^2-1+2.*q(:,3).^2;
    R(2,3,:) = 2.*(q(:,3).*q(:,4)+q(:,1).*q(:,2));
    R(3,1,:) = 2.*(q(:,2).*q(:,4)+q(:,1).*q(:,3));
    R(3,2,:) = 2.*(q(:,3).*q(:,4)-q(:,1).*q(:,2));
    R(3,3,:) = 2.*q(:,1).^2-1+2.*q(:,4).^2;
end

function qConj = quaternConj(q)
    qConj = [q(:,1) -q(:,2) -q(:,3) -q(:,4)];
end

function ab = quaternProd(a, b)
    ab(:,1) = a(:,1).*b(:,1)-a(:,2).*b(:,2)-a(:,3).*b(:,3)-a(:,4).*b(:,4);
    ab(:,2) = a(:,1).*b(:,2)+a(:,2).*b(:,1)+a(:,3).*b(:,4)-a(:,4).*b(:,3);
    ab(:,3) = a(:,1).*b(:,3)-a(:,2).*b(:,4)+a(:,3).*b(:,1)+a(:,4).*b(:,2);
    ab(:,4) = a(:,1).*b(:,4)+a(:,2).*b(:,3)-a(:,3).*b(:,2)+a(:,4).*b(:,1);
end

function v = quaternRotate(v, q)
    [row col] = size(v);
    v0XYZ = quaternProd(quaternProd(q, [zeros(row, 1) v]), quaternConj(q));
    v = v0XYZ(:, 2:4);
end

function euler = rotMat2euler(R)
    // from paper: "Adaptive Filter for a Miniature MEMS Based Attitude and
    // Heading Reference System" by Wang et al, IEEE.
    
    phi = atan2(R(3,2,:), R(3,3,:) );
    theta = -atan(R(3,1,:) ./ sqrt(1-R(3,1,:).^2) );    
    psi = atan2(R(2,1,:), R(1,1,:) );

    euler = [phi(1,:)' theta(1,:)' psi(1,:)'];  
end


function q = rotMat2quatern(R)  
 //wiki URL: 
 //http://en.wikipeRia.org/wiki/Quaternions_anR_spatial_rotation#Fitting_quaternions
 // paper URL:
 // http://www.aiaa.org/content.cfm?pageiR=406&gTable=japaperimport&gIR=4654 
    [row col numR] = size(R);
    q = zeros(numR, 4);
	K = zeros(4,4);    
    for i = 1:numR
        K(1,1) = (1/3) * (R(1,1,i) - R(2,2,i) - R(3,3,i));
        K(1,2) = (1/3) * (R(2,1,i) + R(1,2,i));
        K(1,3) = (1/3) * (R(3,1,i) + R(1,3,i));
        K(1,4) = (1/3) * (R(2,3,i) - R(3,2,i));  
        K(2,1) = (1/3) * (R(2,1,i) + R(1,2,i));
        K(2,2) = (1/3) * (R(2,2,i) - R(1,1,i) - R(3,3,i));
        K(2,3) = (1/3) * (R(3,2,i) + R(2,3,i));
        K(2,4) = (1/3) * (R(3,1,i) - R(1,3,i));   
        K(3,1) = (1/3) * (R(3,1,i) + R(1,3,i));
        K(3,2) = (1/3) * (R(3,2,i) + R(2,3,i));
        K(3,3) = (1/3) * (R(3,3,i) - R(1,1,i) - R(2,2,i));
        K(3,4) = (1/3) * (R(1,2,i) - R(2,1,i));    
        K(4,1) = (1/3) * (R(2,3,i) - R(3,2,i));
        K(4,2) = (1/3) * (R(3,1,i) - R(1,3,i));
        K(4,3) = (1/3) * (R(1,2,i) - R(2,1,i));
        K(4,4) = (1/3) * (R(1,1,i) + R(2,2,i) + R(3,3,i)); 
        [V,D] = eig(K);
        %p = find(max(D));
        %q = V(:,p)';    
        q(i,:) = V(:,4)';
        q(i,:) = [q(i,4) q(i,1) q(i,2) q(i,3)];
    end
end

function euler = quatern2euler(q)
  //from paper: "Adaptive Filter for a Miniature MEMS Based Attitude and
  //Heading Reference System" by Wang et al, IEEE.
    
    R(1,1,:) = 2.*q(:,1).^2-1+2.*q(:,2).^2;
    R(2,1,:) = 2.*(q(:,2).*q(:,3)-q(:,1).*q(:,4));
    R(3,1,:) = 2.*(q(:,2).*q(:,4)+q(:,1).*q(:,3));
    R(3,2,:) = 2.*(q(:,3).*q(:,4)-q(:,1).*q(:,2));
    R(3,3,:) = 2.*q(:,1).^2-1+2.*q(:,4).^2;    
    
    phi = atan2(R(3,2,:), R(3,3,:) );
    theta = -atan(R(3,1,:) ./ sqrt(1-R(3,1,:).^2) );    
    psi = atan2(R(2,1,:), R(1,1,:) );

    euler = [phi(1,:)' theta(1,:)' psi(1,:)']; 
end

function R = euler2rotMat(phi, theta, psi)
    R(1,1,:) = cos(psi).*cos(theta);
    R(1,2,:) = -sin(psi).*cos(phi) + cos(psi).*sin(theta).*sin(phi);
    R(1,3,:) = sin(psi).*sin(phi) + cos(psi).*sin(theta).*cos(phi);
    
    R(2,1,:) = sin(psi).*cos(theta);
    R(2,2,:) = cos(psi).*cos(phi) + sin(psi).*sin(theta).*sin(phi);
    R(2,3,:) = -cos(psi).*sin(phi) + sin(psi).*sin(theta).*cos(phi);
    
    R(3,1,:) = -sin(theta);
    R(3,2,:) = cos(theta).*sin(phi);
    R(3,3,:) = cos(theta).*cos(phi);
end


function R = axisAngle2rotMat(axis, angle)
    kx = axis(:,1);
    ky = axis(:,2);
    kz = axis(:,3);
    cT = cos(angle);
    sT = sin(angle);
    vT = 1 - cos(angle);
    
    R(1,1,:) = kx.*kx.*vT + cT;
    R(1,2,:) = kx.*ky.*vT - kz.*sT;
    R(1,3,:) = kx.*kz.*vT + ky.*sT;
    
    R(2,1,:) = kx.*ky.*vT + kz.*sT;
    R(2,2,:) = ky.*ky.*vT + cT;
    R(2,3,:) = ky.*kz.*vT - kx.*sT;
    
    R(3,1,:) = kx.*kz.*vT - ky.*sT;
    R(3,2,:) = ky.*kz.*vT + kx.*sT;
    R(3,3,:) = kz.*kz.*vT + cT;
end


function q = axisAngle2quatern(axis, angle)
    q0 = cos(angle./2);
    q1 = -axis(:,1)*sin(angle./2);
    q2 = -axis(:,2)*sin(angle./2);
    q3 = -axis(:,3)*sin(angle./2); 
    q = [q0 q1 q2 q3];
end

