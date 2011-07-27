#!/bin/sh -ex
#COMMIT="--commit 0"
./ingest_data.py -i amys_country_cheddar_bowl -d "Amy's frozen dinner. Country Cheddar Bowl" \
  -b data/amys_country_cheddar_bowl.bag \
  $COMMIT \
  food tod kitchen box
./ingest_data.py -i band_aid_plastic_strips -d "Band-aid brand bandages. Plastic strip 60 pack. Johnson&Johnson." \
  -b data/band_aid_plastic_strips.bag \
  $COMMIT \
  tod box "first-aid"
./ingest_data.py -i delmonte_peas_carrots -d "Can of peas and carrots." \
  -b data/delmonte_peas_carrots.bag \
  $COMMIT \
  can food kitchen veggies tod
./ingest_data.py -i goodearth_original_tea -d "A box of caffeine free original tea. Sweet & Spicy. 18 bags." \
  -b data/goodearth_original_tea.bag \
  $COMMIT \
  tea food tod box kitchen
./ingest_data.py -i jello_strawberry -d "Box of strawberry jello mix." \
  -b data/jello.bag \
  $COMMIT \
  food kitchen box tod jello
./ingest_data.py -i paneer_tikka_masala_spinach_trader_joes -d "Trader joes box of paneer tikka masala with spinach and basmati rice." \
  -b data/paneer_tikka_masala.bag \
  $COMMIT \
  food kitchen box tod
./ingest_data.py -i spaghettios_meatballs -d "Can of spaghettios with meatballs. 14.75 oz." \
  -b data/spaghettios.bag \
  $COMMIT \
  can food tod kitchen
./ingest_data.py -i tazo_organic_chai_box -d "Tazo chai box tea. 1.9 oz box." \
  -b data/tazo_organic_chai_box.bag \
  $COMMIT \
  box tea food kitchen

