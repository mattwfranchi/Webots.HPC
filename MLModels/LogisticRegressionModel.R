#!/usr/bin/env Rscript

# Rebecca Kahn

rm(list = ls())

library(ISLR)
library(dplyr)
library(ggplot2)

CAV <- read.csv("data_1623082849_871502.csv", na.strings= c("", " "))

#CAV$Risk_Status <- NULL
CAV$TTCb <- NULL
CAV$TTCf <- NULL


normalized <- (CAV$acce_change-min(CAV$acce_change))/(max(CAV$acce_change-min(CAV$acce_change)))




fit <- glm((X + Speed_Merge + Acc_Merge + Steering_Merge + Speed_Lead + Acc_Lead + 
             Gap_Lead + Acce_Lag + Gap_Lag + Remaining_Distance + Num_Adj_Vehs_Start +
             Num_Adj_Vehs_End + Risk_Status) ~ poly(normalized, 1), data=CAV)
coef(summary(fit))
summary(fit)
