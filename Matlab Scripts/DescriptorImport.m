%March 2018
%Nathan I N Daley

%Import program for PFH and FPFH descriptors in MatLAB

%Run and select a file in the target folder, all files will be imported
%separately as vectors and as a single concatenated matrix at the end.

clear
clc
%Variables;
global PATH;
global FILE;
posHolder={};
velHolder={};
accelHolder={};


%Variables to Be Set
EndNumber=4; %This number determines the amount of points that are plotted in the final plot for each run.



%Add the location of the current script to matlab for swallow_csv/
startingDir=pwd;
addpath(startingDir);


[FileName,PathName]=uigetfile('*.csv','Identify a CSV file');
PATH=PathName;
FILE=FileName;
fullFilePath=strcat(PathName,FileName);

%Send the file information of the files to an array.
resetdir=PATH;
allfiles=dir(strcat(PATH,'*.csv'));
%Retreive list of all files within directory ending in .csv
lengthfiles=length(allfiles);

for i=1:lengthfiles %Operations to occur on each file
    %Read in all files to an array
   
    FILE=allfiles(i).name;
    filenocsv=FILE(1:end-4);
    
    genvarname(filenocsv);
    
    tic
    [numbers] = swallow_csv(strcat(PATH,FILE), '"', ',', '\');
    toc
    %Obtain dimensions of the input file
    
    [rows,cols]=size(numbers);
    eval([filenocsv '= numbers']);
    
    numpath=double(PATH);
    
    slashid=find(numpath==92);
    folderend=slashid(end);
    folderbegin=slashid(end-1);
    Concatname=PATH((folderbegin+1):(folderend-1));
    
    
    
end