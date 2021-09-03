# Simple classification model 

rm(list = ls())

library(plyr)
library(tidyverse)
library(stringr)
library(dplyr)
library(readr)

substrRight <- function(x , n)
{
  substr(x, nchar(x)-n+1, nchar(x))
}

#that works individually
#run.data <- read.csv(all_files[1], header=TRUE,na.strings=c("", " "), skip = 100)

all_files <- list.files(pattern="*True.csv|*False.csv", full.names = TRUE)
merged.data <- data.frame()



for(x in 1:length(all_files))
{
  headers <- names(read.csv(all_files[x], nrows=1))
  run.data <- read.csv(all_files[x], header=F, col.names = headers, na.strings=c("", " "), skip=100)
  collision <- str_sub(substrRight(all_files[x],9),1,5)
  collision <- ifelse(collision=="False",0,1)
  run.data <- cbind(run.data,collision)
  
  merged.data <- rbind(merged.data,run.data)
  
}

write.csv(merged.data, "~Downloads\\output.csv", row.names = FALSE)
