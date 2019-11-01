Name: Khurem Dehri

Mass Spring Physical Simulation of Cloth:
----------------------------------------
Basic cloth simulation with the ability to tear and disturb.

To tear the cloth, just left-click over any portion of the cloth.

To disturb, press and hold 'Ctrl' and then left-click any portions to disturb and drag.

If there are bugs during runtime(hint: there will be), press 'R' to reset cloth. If bugs are still not alleviated, press Esc and start program over

![Output sample](https://github.com/Khurem/cloth_sim/gifs/output_opt.gif)

TO MAKE AND RUN:
----------------
tar xf <YOUR.tar.gz>
cd mass_spring
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8 (or make cloth)
./bin/cloth

BUGS AND WARNING INFORMATION:
-----------------------------
When running, the cloth should generate properly and drape down. Disturbing it using the Ctrl + Left-Click method should be stable.
However, when you begin to tear, things get unpredictable. Be it an issue with the initialization of the two separated points or some error in 
calculating force, tearing can cause bugs in the cloths stability. In this, refer to the last statement above the warning header and attempt to 
restart. 
