# coding:utf-8 
require 'csv'

csv = CSV.open("_result_whole_" + ARGV[0]+ ".csv" ,"w") 
#outfile = File.open('_result_whole.csv', 'a')
#CSV::Writer.generate(outfile) do |writer|
#csv = CSV.open("_result_whole_" + $ARGV[0] + ".csv","w")
#csv << ["計算時間","シミュレーション内時間","小型車発生台数","大型車発生台数","事故数","エラー率"]
rates = ["10","20","30"]
rates.each do |r|
  zeros = " .txt"
  file = open("./_stat_accident" + r + zeros)
  count = 0
  file.each do |line|
    count+=1
  end
  nLine = count
  p "==============="
  p "nLine:" + nLine.to_s
  p "==============="   
  count = 0

  file2 = open("./_stat_accident" + r + zeros) 
  file2.each do |line|
    count+=1
    p count
    if count == nLine-1 
      new_line = [line.gsub(",\n","") + "," + r]
      csv << new_line 
    end
  end
end
