#!/usr/bin/env Rscript

# Matt Franchi
# R Implementation of ARIMA Model to analyze our traffic simulation data

# Libraries
library(dse)
library(ggplot2)
library(reshape)

# Update Working Directory to CAVSITAW Repo
setwd("~/Repos/CAVSITAW")

# File Input 
filename <- file.choose()
data.raw <- read.csv(filename,na.strings=c(""," "))
data.output <- data.raw$Risk_Status
data.input <- data.raw
data.input$Risk_Status <- NULL

# Some Intro Plots
# ACC_LEAD SCATTER PLOT
ggplot(data.raw,aes(x=X,y=Acc_Lead)) + geom_point()
ggsave("acc_lead.png",units="in",width=24,height=18)

# ACC_LAG SCATTER PLOT
ggplot(data.raw,aes(x=X,y=Acce_Lag)) + geom_point()
ggsave("acc_lag.png",units="in",width=24,height=18)

# ACC_MERGE SCATTER PLOT
ggplot(data.raw,aes(x=X,y=Acc_Merge)) + geom_point()
ggsave("acc_merge.png",units="in",width=24,height=18)

# SPEED_LEAD SCATTER PLOT
ggplot(data.raw,aes(x=X,y=Speed_Lead)) + geom_point()
ggsave("speed_lead.png",units="in",width=24,height=18)

# SPEED_LAG SCATTER PLOT
ggplot(data.raw,aes(x=X,y=Speed_Lag)) + geom_point()
ggsave("speed_lag.png",units="in",width=24,height=18)

# SPEED_MERGE SCATTER PLOT
ggplot(data.raw,aes(x=X,y=Speed_Merge)) + geom_point()
ggsave("speed_merge.png",units="in",width=24,height=18)

# REMAINING_DISTANCE SCATTER PLOT
ggplot(data.raw,aes(x=X,y=Remaining_Distance)) + geom_point()
ggsave("remaining_distance.png",units="in",width=24,height=18)

# SPEED COMPARISON SCATTER PLOT
speeds <- c("X","Speed_Merge","Speed_Lead","Speed_Lag")
data.speeds <- data.raw[speeds]

data.speeds.m <- melt(data.speeds,id.vars="X")

ggplot(data.speeds.m,aes(X, value, colour = variable)) + 
  geom_point() +
  scale_colour_manual(values = c("red","blue","green"))
ggsave("speeds.png", units="in",width=24,height=18)
# Data Processing
# WITH DSE LIBRARY
data.ts <- TSdata(input=data.input, output=data.output)





