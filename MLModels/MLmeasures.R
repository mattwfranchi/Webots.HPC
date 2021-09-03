# With xgboost model, find accuracy

rm(list = ls())

library(xgboost)

filename = file.choose()
cavsitaw.data <- read.csv(filename, na.strings=c(""," "))
cavsitaw.data$Risk_Status <- NULL

acce_change = cavsitaw.data$acce_change
label <- as.integer(cavsitaw.data$acce_change) -1
cavsitaw.data$acce_change = NULL

n = nrow(cavsitaw.data)
train.index = sample(n, floor(0.75*n))
train.data = as.matrix(cavsitaw.data[train.index,])
train.label = label[train.index]
test.data = as.matrix(cavsitaw.data[-train.index,])
test.label = label[-train.index]


xgb.train = xgb.DMatrix(data=train.data,label=train.label)
xgb.test = xgb.DMatrix(data=test.data, label=test.label)

num_class = length(levels(acce_change))
params = list(
  booster="gbtree",
  eta=0.3,
  max_depth=5,
  gamma=3,
  subsample=0.75,
  colsample_bytree=1,
  objective="reg:squarederror", #multi:softprob
  eval_metrix="mlogloss",
  num_class=num_class
)


# Train model
xgb.fit=xgb.train(
  params=params,
  data=xgb.train,
  nrounds=10000,
  nthreads=1,
  early_stopping_rounds=10,
  watchlist=list(vall=xgb.train,val2=xgb.test),
  verbose=0
)

xgb.fit


# Predict new outcomes
xgb.pred = predict(xgb.fit, test.data, reshape=T)
xgb.pred = as.data.frame(xgb.pred)
colnames(xgb.pred) = levels(acce_change)

# Find class w highest probability for each predicition
xgb.pred$prediction = apply(xgb.pred, 1, function(x) colnames(xgb.pred)[which.max(x)])
xgb.pred$label = levels(acce_change)[test.label+1]

# Accuracy
result = sum(xgb.pred$prediction==xgb.pred$label)/nrow(xgb.pred)
print(paste("Final Accuracy =", sprintf("%1.2f%%", 100*result)))
