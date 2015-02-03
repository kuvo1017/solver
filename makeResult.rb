#-*- encoding:utf-8 -*-
#このファイルはresult201~/で使用する
#res1,res2,res3から結果を取ってきて一つのcsvを創る
require 'csv'

def files(path ,type)
  files = []
  Dir::glob( path + "/*" ).each {|fname|
    if FileTest.directory?( fname ) 
      files( fname, type )
    else
      if fname.include?("stat_accident_") and fname.include?(type)
	files.push(fname)
	  puts fname
      end
    end
  }
  return files
end  

def filesTrip()
  files = []
  Dir::glob("./res1/*").each {|fname|
    if FileTest.directory?( fname ) 
      files( fname, type )
    else
      if fname.include?("vehicleTrip") 
	files.push(fname)
	  puts fname
      end
    end
  }
  return files
end  
 
dirs = ["REAR","PASSING","LR" ,"SHIFT" ,"HEAD","OKAYAMA"] 
parentDirs = ["res1","res2","res3"]

dirs.each do |d|
  csv = CSV.open( "./" + "_okayama_" + d + ".csv", "a") 
  csv << ["calc-time,sim-time,traffic-volume(small),traffic-volume(large),number-accident,error-rate,accident-rate"]
  parentDirs.each do |p|
  #ARGV[0]はどのディレクトリかが入る e.g)res1 
    files = files( "./" + p ,d )
    files.each do |f|
      file = open(f)
      count = 0
      file.each do |line|
        count+=1
	end
	nLine = count
=begin
	p "==============="
	p "nLine:" + nLine.to_s
	p "==============="   
=end
	count = 0

        file2 = open(f) 
        file2.each do |line|
          count+=1
#          p count
          if count == nLine 
	    f_rev = f.gsub("./" + p + "/_stat_accident_", "")
  	    f_rev = f_rev.gsub(".txt", "")
	    f_rev = f_rev.gsub("NOLOOK_REAR_", "")
	    f_rev = f_rev.gsub("ARROGANCE_PASSING_", "")
	    f_rev = f_rev.gsub("ARROGANCE_LR_", "")
	    f_rev = f_rev.gsub("NOLOOK_SHIFT_", "")
	    f_rev = f_rev.gsub("NOLOOK_HEAD_", "")
	    f_rev = f_rev.gsub("OKAYAMA_", "")
 	    puts f_rev
	    new_line = [line.gsub(",\n","") + "," + f_rev]
	    csv << new_line 
	  end
        end
      end   
   end
end

csvTrip = CSV.open( "./" + "_trip.csv", "a") 
ft = filesTrip()
p "!!!!!!!!"
p ft.size
ft.each do |f|
  file = open(f)
  count = 0
  file.each do |line|
    count+=1
  end
  nLine = count
  count = 0

  
  file2 = open(f) 
  totalTrip = 0
  count = 0
  file2.each do |line|
    l = line.split(",") 
    p l[0]
    totalTrip += l[1].to_f
    if count == nLine -2
      csvTrip << [f,totalTrip] 
    end
    count += 1
  end
end
  
