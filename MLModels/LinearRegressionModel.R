#!/usr/bin/env Rscript

# Rebecca Kahn
# Simple linear regression based on risk status compared to all other variables

rm(list = ls())
#filename = file.choose()
#data <- read.csv(filename,na.strings=c(""," "))


data <- read.csv("data_1623082849_871502.csv", na.strings= c("", " "))

dim(data)
na.omit(da)

data$TTCb <- NULL
data$TTCf <- NULL
#data$Risk_Status <- NULL

#attach(data)

#dim(data)


library(ggplot2)
library(boot)

normalized <- (data$acce_change-min(data$acce_change))/(max(data$acce_change-min(data$acce_change)))

data$acce_change <- NULL

lm.fit <- lm(normalized ~ ., data)
summary(lm.fit)

#library( broom )
#a <- summary(lm.fit)
#write.csv( tidy( a ) , "coefs.csv" )
#write.csv( glance( a ) , "an.csv" )
