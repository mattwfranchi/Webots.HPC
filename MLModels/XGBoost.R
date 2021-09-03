# CAV Situational Awareness
# Matt Franchi and Rebecca Kahn

rm(list = ls())

require(xgboost)
require(caTools) 
require(ggplot2)
require(DiagrammeR)
require(circlize)

# Dataset Loading
data_filename = file.choose()
cavsitaw.data <- read.csv(data_filename, na.strings=c(""," "))
#train_filename = file.choose()
#cavsitaw.data.train <- read.csv(train_filename,na.strings=c(""," "))
cavsitaw.data.train <- cavsitaw.data[1:489150, ]
cavsitaw.data.train$Risk_Status <- NULL

#test_filename = file.choose()
#cavsitaw.data.test <- read.csv(test_filename, na.strings=c("", " "))
cavsitaw.data.test <- cavsitaw.data[489151:length(cavsitaw.data), ]
cavsitaw.data.test$Risk_Status <- NULL


#cavsitaw.data.train <- cavsitaw.data[train.index, ]
#cavsitaw.data.test <- cavsitaw.data[-train.index, ]

cavsitaw.label.train <- cavsitaw.data.train$collision
cavsitaw.label.test <- cavsitaw.data.test$collision

#range01 <- function(x, ...){(x - min(x, ...)) / (max(x, ...) - min(x, ...))}
#cavsitaw.label.train <- range01(cavsitaw.label.train)
#cavsitaw.label.test <- range01(cavsitaw.label.test)

cavsitaw.data.train$collision <- NULL
cavsitaw.data.test$collision <- NULL

cavsitaw.data.train <- data.matrix(cavsitaw.data.train)
cavsitaw.data.test <- data.matrix(cavsitaw.data.test)

summary(cavsitaw.label.train)

# Model Creation
dtrain <- xgb.DMatrix(data = cavsitaw.data.train, label=cavsitaw.label.train)
dtest <- xgb.DMatrix(data = cavsitaw.data.test, label=cavsitaw.label.test)

cavsitaw.watchlist <- list(train=dtrain, test=dtest)
bst <- xgb.train(data = dtrain,
                 max.depth = 10,
                 eta = .3,
                 nthread = 4,
                 nrounds = 25,
                 num_parallel_tree = 5,
                 watchlist = cavsitaw.watchlist,
                eval.metric="map",
#                 early_stopping_rounds = 5,
                 objective = "binary:logistic")
#eval.metric = "error" -> then 1-_ for accuracy

# data before with first hundred lines in dataset
# map: [25]	train-map:0.849561	test-map:0.557562 
# error: [25]	train-error:0.039979	test-error:0.078292 
# accuracy: train:0.960021  test-error:0.921708

# data results when first hundred lines taken out of dataset

# map: [25]	train-map:0.951191	test-map:0.951191 
# error: [25]	train-error:0.020664	test-error:0.020665 
# accuracy: train:0.979336  test:0.979335

# Prediction
pred <- predict(bst,cavsitaw.data.test)
#**********
#*
library(MASS)
train.data <- as.data.frame(t(cavsitaw.data.train))
fit <- lda(acce_change ~., data = train.data)
predicted.classes <- pred
observed.classes <- cavsitaw.data.test


accuracy <- mean(observed.classes == predicted.classes)

accuracy

error <- 1 - accuracy

print(length(pred))

print(head(pred))



# Measuring Model Performance
residuals = cavsitaw.label.test - pred
RMSE = sqrt(mean(residuals^2))

labels_mean = mean(cavsitaw.label.test)
tss = sum((cavsitaw.label.test - labels_mean)^2)
rss = sum(residuals^2)

rsq = 1 - (rss/tss)
cat('The R-square of the test data is ', round(rsq,3), '\n')

# Plots
#test_times <- head(cavsitaw.data$X,100)
#df <- data.frame(test_times,pred,cavsitaw.label.test)
#ggplot(df, aes(x=test_times)) + 
  #geom_line(aes(y = pred), color = "darkred", linetype="twodash") + 
  #geom_line(aes(y = cavsitaw.label.test), color="steelblue") +
  #ggtitle("Predicted v. Actual Jerk [One Simulation Run]") +
  #theme(plot.title = element_text(hjust = 0.5)) +
  #xlab("Time Elapsed [s]") +
  #ylab("Jerk [m/s^3]")

plot(cavsitaw.data.test[1:360,1],head(cavsitaw.label.test,360))
for (i in 0:2)
{
  points(cavsitaw.data.test[360*i:360*(i+1)],cavsitaw.label.test[360*i:360*(i+1)],col=rand_color(1,luminosity="random"))
}

#plot(cavsitaw.data.test[,1],cavsitaw.label.test)
#points(head(pred,360),col="green")
xgb.plot.tree(model=bst,trees=1)

# Importance Matrix
cavsitaw.importance <- xgb.importance(model = bst)
print(cavsitaw.importance)
xgb.plot.importance(importance_matrix = cavsitaw.importance)



# Calculate accuracy, precision, recall, and error
