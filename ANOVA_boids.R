## -----------------------------------------------------------------------------------------------------------------------------------------------------

require(tidyverse)
require(car)
require(DescTools)
require(dplyr)
require(ggpubr)
require(kableExtra)
require(data.table)
library(ez)
library(car)





## -----------------------------------------------------------------------------------------------------------------------------------------------------

df_area <- read.csv("area vs fps - R.csv", header = TRUE)

#reading a data frame of the given dataset


#conducting one-way between-subject ANOVA on algorithm and fps
anovatest <- aov(df_area$fps ~ as.factor(df_area$algorithm), data = df_area)
  
summary(anovatest)

# Conducting Tukey's HSD 
tHSD <- TukeyHSD(anovatest)
print(tHSD)
# Tukey's SCI plotted
plot(tHSD)








## -----------------------------------------------------------------------------------------------------------------------------------------------------

df_boids <- read.csv("boids vs fps - R.csv", header = TRUE)

#reading a data frame of the given dataset


#conducting one-way between-subject ANOVA on algorithm and fps
anovatest <- aov(df_boids$fps ~ as.factor(df_boids$Algorithm), data = df_boids)
  
summary(anovatest)

# Conducting Tukey's HSD 
tHSD <- TukeyHSD(anovatest)
print(tHSD)
# Tukey's SCI plotted
plot(tHSD)




