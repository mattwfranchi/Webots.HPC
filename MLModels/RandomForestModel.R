# Rebecca

rm(list = ls())
library(randomForest)

CAV <- read.csv("merged_data_1622220445_50.csv", na.strings= c("", " "))

CAV$TTCb <- NULL
CAV$TTCf <- NULL

attach(CAV)

set.seed(1)
N <- nrow(CAV)
train <- sample(1:N, N/2)
test <- seq(1:N)[-train]

cav.test <- CAV[test, "Risk_Status"]

P <- ncol(CAV) - 1
bag.cav <- randomForest(Risk_Status~., data=CAV, subset=train, mtry=P, importance=TRUE, na.action=na.roughfix)
bag.cav

yhat.rf <- predict(bag.cav, CAV[test,])
mean((yhat.rf-cav.test)^2)


importance(bag.cav)
varImpPlot(bag.cav)


#boosting
library(gbm)
set.seed(1)

boost.cav <- gbm(Risk_Status~., data=CAV[train,], distribution="gaussian", n.trees = 5000, interaction.depth=4)
summary(boost.cav)

par(mfrow=c(1,2))
plot(boost.cav, i="Acc_Merge")
plot(boost.cav, i="X")

yhat.boost <- predict(boost.cav, newdata = CAV[test,], n.trees=5000)
mean((yhat.boost-cav.test)^2)


#boost.cav <- gbm(Risk_Status~., data=CAV[train,], distribution = "gaussian", n.trees=5000, interaction.depth = 4, shrinkage = 0.2, verbose=F)
#yhat.boost <- predict(boost.cav, CAV[test,], n.trees=5000)
#mean((yhat.boost-cav.test)^2)
