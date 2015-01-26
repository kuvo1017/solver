rm _okayama*
for d in "res1" "res2" "res3"
  do
  cd $d
  rm _*
  sh cp-result.sh
  cd ../
  done
ruby ./makeResult.rb 
mkdir simulations
cp -r /calc/medusa06/kubo/advmates/trunk/simulations/rear_error/ ./simulations/rear_error
cp -r /calc/medusa07/kubo/advmates/trunk/simulations/passing_error/ ./simulations/passing_error 
cp -r /calc/medusa08/kubo/advmates/trunk/simulations/lr_error/ ./simulations/lr_error 
cp -r /calc/medusa09/kubo/advmates/trunk/simulations/shift_error/ ./simulations/shift_error/ 
cp -r /calc/medusa10/kubo/advmates/trunk/simulations/head_error/ ./simulations/head_error/ 
