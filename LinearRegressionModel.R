#!/usr/bin/env Rscript

# Rebecca Kahn
# Simple linear regression based on risk status compared to all other variables


rm(list = ls())

filename = file.choose()
data <- read.csv(filename,na.strings=c(""," "))
# data <- read.csv("/zfs/dicelab/merging_sim_output/data_1620826514.csv")
dim(data)
d <- na.omit(data)


library(ggplot2)
library(boot)


data$X <- NULL
data$TTCf <- NULL
data$TTCb <- NULL
#data$Acc_Lead <- NULL 
#data$Speed_Lead <- NULL
#data$Gap_Lead <- NULL
data$Num_Adj_Vehs_End <- NULL
data$Num_Adj_Vehs_Start <- NULL
#data$Acc_Merge <- NULL
#data$Acce_Lag <- NULL
#data$Remaining_Distance <- NULL
summary(data)

# df = data.frame(data$TTCf, data$TTCb, data$Risk_Status)
# lm.fit <- lm(data$Risk_Status ~ ., df)
# summary(lm.fit)
# R-squared is 1 

png("myplot.png")
p <- ggplot(data, aes(Risk_Status)) +
  geom_histogram()
print(p)

lm.fit <- lm(Risk_Status ~ ., data)
summary(lm.fit)

library( broom )
a <- summary(lm.fit)
summary(lm.fit)
write.csv( tidy( a ) , "coefs.csv" )
write.csv( glance( a ) , "an.csv" )


#gap lead, speed lead, acce lag, gap lag, speed lag

#lm(data$TTCf ~ data$Speed_Lead)
#abline(1.04141, -0.04011)
#plot(data$TTCf, data$Speed_Lead)
#abline(lm(data$TTCf ~ data$Speed_Lead))


# data_1620266883.csv
# Multiple R-squared:  0.5385,	Adjusted R-squared:  0.5261

# data_1620236450.csv
#Speed_Lead and Gap_Lead at 0.05 or less (shows significance)
# Multiple and adjusted above 0.95 (Substantial R-squared value)