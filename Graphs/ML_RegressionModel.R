# Rebecca Kahn
# Logistic regression classification model, using 5-fold to measure error
# estimate a SVM -> EDA between predictor and response


rm(list = ls())

data <- read.csv("tp1618317296.csv")
dim(data)
d <- na.omit(data)

set.seed(266)

boxplot(d$Current.State)

# Min = 0.000, Mean = 4.214, Max = 15.000
summary(d$Current.State)


N <- nrow(d)
train <- data(1:N, 266)
test <- seq(1:N)[-train]


library(randomForest)


P <- ncol(d)-1

class(d$Current.State)
# regression -> P/3
rf.regressionE <- randomForest(Current.State~., data=d, subset=train, mtry = P/3, 
                               keep.forest=TRUE, importance=TRUE)


d.test <- d[test, "Current.State"]
yhat.regressionE <- predict(rf.regressionE, d[test,])




# N <- nrow(d)
# train <- d(1:N, 20)
# test <- seq(1:N)[-train]

# d.test <- d[test, "Current.State"]

# bag.boost <- gbm(Current.State~., data=d[train,], distribution = "gaussian", 
#                 n.trees = 1000, interaction.depth=5, bag.fraction=1, 
#                 shrinkage=0.001)
# yhat.bag <- predict(bag.boost, d[test,])
# mean((yhat.bag- d.test)^2)