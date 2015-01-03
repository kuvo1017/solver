require 'csv'


def files( path )
  files = []
  Dir::glob( path + "/*" ).each {|fname|
    if FileTest.directory?( fname ) 
      files( fname )
    else
      if fname.include?("stat_accident_")
	files.push(fname)
	  puts fname
      end
    end
  }
  return files
end  

dirs = ["result-rear","result-passing","result-lr" ,"result-shift" ,"result-head"] 

dirs.each do |d|
csv = CSV.open("_result_whole_" + d + ".csv" ,"w") 
 
files = files("./" + d + "/")

files.each do |f|
  file = open(f)
  count = 0
  file.each do |line|
    count+=1
  end
  nLine = count
  p "==============="
  p "nLine:" + nLine.to_s
  p "==============="   
  count = 0

  file2 = open(f) 
  file2.each do |line|
    count+=1
    p count
    if count == nLine 
      new_line = [line.gsub(",\n","") + "," + f]
      csv << new_line 
    end
  end
end   
