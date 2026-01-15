/home/sihoon/Applications/flightgear.AppImage \
  --generic=serial,bi,50,/dev/ttyACM0,115200,phantom_protocol \
  --in-air \
  --altitude=3000 \
  --vc=100 \
  --prop:/controls/engines/engine/throttle=1.0 \
  --prop:/controls/engines/engine/mixture=1.0 \
  --prop:/sim/model/c172p/damage_enabled=false \
  --prop:/sim/freeze/fuel=true \
  --timeofday=noon
