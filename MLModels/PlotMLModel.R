# Plot test/training of ml classification model


rm(list = ls())

library(ggplot2)

data1 = file.choose()
leastTrain.midhighTest <- read.csv(data1, na.strings=c(""," "))

data2 = file.choose()
midhighTrain.leastTest <- read.csv(data2, na.strings=c(""," "))


Epoch <- leastTrain.midhighTest$Epoch
TrainErrorRate.least <- leastTrain.midhighTest$TrainError
TrainErrorRate.midhigh <- midhighTrain.leastTest$TrainErrorRate

dfTrainError <- data.frame(Epoch, TrainErrorRate.least, TrainErrorRate.midhigh)

TestErrorRate.least <- leastTrain.midhighTest$TestError
TestErrorRate.midhigh <- midhighTrain.leastTest$TestErrorRate

dfTestError <- data.frame(Epoch, TestErrorRate.least, TestErrorRate.midhigh)

trainPlot <- 0
trainPlot <- ggplot(dfTrainError, aes(x = Epoch))
trainPlot <- trainPlot + geom_line(aes(y=TrainErrorRate.least, color="Least to MidHigh Traffic"))
trainPlot <- trainPlot + geom_line(aes(y=TrainErrorRate.midhigh, color="MidHigh to Least Traffic"))
trainPlot <- trainPlot + ylab("Train Error Rate") + xlab("Epoch")
trainPlot

testPlot <- 0
testPlot <- ggplot(dfTestError, aes(x = Epoch))
testPlot <- testPlot + geom_line(aes(y=TestErrorRate.least, color="Least to MidHigh Traffic"))
testPlot <- testPlot + geom_line(aes(y=TestErrorRate.midhigh, color="MidHigh to Least Traffic"))
testPlot <- testPlot + ylab("Test Error Rate") + xlab("Epoch")
testPlot


      
