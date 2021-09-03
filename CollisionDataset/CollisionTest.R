# Goes through all the csv files in the directory and lists the ones w a collision

rm(list = ls())

library(plyr)
library(tidyverse)
library(stringr)


list_of_files = list.files(pattern="*.csv", full.names = TRUE)


print(length(list_of_files))

for (i in 1:length(list_of_files)){
  Data <- read.csv(list_of_files[i])
  df <- list_of_files %>%
    setNames(nm = list_of_files[i])
  if (nrow(Data) < 300){
    d <- data.frame(Column1 = c(list_of_files[i]))
      
  }
}

write.csv(d, "~/Documents/BMWResearch/CollisionDataset/collisions.csv")


