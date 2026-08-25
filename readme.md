use ml as conda baseline

![alt text](image.png)

![alt text](image-1.png)


![alt text](image-2.png)

   
                     PID         LQR         LQI          L1         MPC
drop              0.1372      0.2233      0.0824      0.0282      0.3222   <- rms err[m]
                  0.0904      0.3113      0.0299      0.0129      0.4415   <- final err[m]
                     0.0         0.0         0.0         0.0         0.0   <- saturated %

climb             0.8643      0.8206      0.8507      0.7918      0.8187   <- rms err[m]
                  0.1683      0.0000      0.0755      0.0030      0.0012   <- final err[m]
                     4.0         3.4         5.9         4.7         2.6   <- saturated %

gust              0.1128      0.0975      0.0897      0.0342      0.1522   <- rms err[m]
                  0.0121      0.0201      0.0378      0.0388      0.0536   <- final err[m]
                     0.0         0.0         0.0         0.0         0.0   <- saturated %

noise             0.0007      0.0006      0.0008      0.0004      0.0006   <- rms err[m]
                  0.0005      0.0003      0.0003      0.0001      0.0004   <- final err[m]
                     0.0         0.0         0.0         0.0         0.0   <- saturated %

ratelimit         0.8991      0.8027      0.8996      0.7968      0.8652   <- rms err[m]
                  0.3375      0.2379      0.5481      0.2519      0.3014   <- final err[m]
                     6.0         4.9        12.2         5.1         1.2   <- saturated %

(ml) shubhamsingh@MacBookAir Controllers % 