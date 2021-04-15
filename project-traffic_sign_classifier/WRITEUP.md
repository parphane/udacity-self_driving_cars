# **Traffic Sign Recognition** 

## Writeup

### You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---

**Build a Traffic Sign Recognition Project**

The goals / steps of this project are the following:
* Load the data set (see below for links to the project data set)
* Explore, summarize and visualize the data set
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./writeup/dataset_repartition.png "Dataset initial repartition"
[image2]: ./writeup/dataset_sample.png "Sample of each dataset label"
[image3]: ./writeup/dataset_augmented.png "Sample of each augmented label"
[image4]: ./writeup/dataset_augmented_repartition.png "Dataset augmented repartition"
[image5]: ./writeup/dataset_grayscale.png "Sample of each grayscale label"
[image6]: ./writeup/dataset_shuffled_repartition.png "Dataset shuffled repartition"
[image7]: ./writeup/lenet_43.png "LeNet 43 classes"
[image8]: ./writeup/dataset_german.png "German dataset"
[image9]: ./writeup/dataset_german_prediction.png "German dataset prediction"
[image10]: ./writeup/dataset_german_certainty.png "German dataset certainty"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/481/view) individually and describe how I addressed each point in my implementation.  

---

### Side notes

* Imported all files from the notebook using the zip function
  * Jupyter > New > Terminal
  * zip -r CarND-Traffic-Sign-Classifier-Project.zip ./
* Used a local Anaconda / Jupyter notebook to work
  * Environments > <environment> > Open terminal
    * pip install tensorflow==1.3
* Changed default browser and path for Jupyter terminal: "New > (Other) Terminal"
  * >jupyter notebook --generate-config
  * Jupyter configuration: C:\Users\<USER>\.jupyter\jupyter_notebook_config.py
  * Uncomment the following line and complete with appropriate path
  * c.NotebookApp.browser = u'C:/Program Files/Mozilla Firefox/firefox.exe %s'
  * c.NotebookApp.notebook_dir = 'D:/workspaces'
* Remove autoclosing and add lane numbers
  * from notebook.services.config import ConfigManager
  * c = ConfigManager()
  * c.update('notebook', {"CodeCell": {"cm_config": {"autoCloseBrackets": False, "lineNumbers": True}}})  


* Worked on my private gitHub repository
  * https://github.com/parphane/udacity-self_driving_cars

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf. You can use this template as a guide for writing the report. The submission includes the project code.

You're reading it! And here is a link to my project code: 
 * [https://github.com/parphane/udacity-self_driving_cars/tree/master/project-traffic_sign_classifier](https://github.com/parphane/udacity-self_driving_cars/tree/master/project-traffic_sign_classifier)

### Data Set Summary & Exploration

#### 1. Provide a basic summary of the data set. In the code, the analysis should be done using python, numpy and/or pandas methods rather than hardcoding results manually.

I used the numpy library to calculate summary statistics of the traffic
signs data set:
* The size of training set is 34799
* The size of the validation set is 4410
* The size of test set is 12630
* The shape of a traffic sign image is 32x32x3 (RGB)
* The number of unique classes/labels in the data set is 43

#### 2. Include an exploratory visualization of the dataset.

Here is an exploratory visualization of the data set:

| Count  | ID | Sign |
| ------------- | ------------- | ------------- |
|180 | 0 | Speed limit (20km/h) |
| 1980 | 1 | Speed limit (30km/h) |
| 2010 | 2 | Speed limit (50km/h) |
| 1260 | 3 | Speed limit (60km/h) |
| 1770 | 4 | Speed limit (70km/h) |
| 1650 | 5 | Speed limit (80km/h) |
| 360 | 6 | End of speed limit (80km/h) |
| 1290 | 7 | Speed limit (100km/h) |
| 1260 | 8 | Speed limit (120km/h) |
| 1320 | 9 | No passing |
| 1800 | 10 | No passing for vehicles over 3.5 metric tons |
| 1170 | 11 | Right-of-way at the next intersection |
| 1890 | 12 | Priority road |
| 1920 | 13 | Yield |
| 690 | 14 | Stop |
| 540 | 15 | No vehicles |
| 360 | 16 | Vehicles over 3.5 metric tons prohibited |
| 990 | 17 | No entry |
| 1080 | 18 | General caution |
| 180 | 19 | Dangerous curve to the left |
| 300 | 20 | Dangerous curve to the right |
| 270 | 21 | Double curve |
| 330 | 22 | Bumpy road |
| 450 | 23 | Slippery road |
| 240 | 24 | Road narrows on the right |
| 1350 | 25 | Road work |
| 540 | 26 | Traffic signals |
| 210 | 27 | Pedestrians |
| 480 | 28 | Children crossing |
| 240 | 29 | Bicycles crossing |
| 390 | 30 | Beware of ice/snow |
| 690 | 31 | Wild animals crossing |
| 210 | 32 | End of all speed and passing limits |
| 599 | 33 | Turn right ahead |
| 360 | 34 | Turn left ahead |
| 1080 | 35 | Ahead only |
| 330 | 36 | Go straight or right |
| 180 | 37 | Go straight or left |
| 1860 | 38 | Keep right |
| 270 | 39 | Keep left |
| 300 | 40 | Roundabout mandatory |
| 210 | 41 | End of no passing |
| 210 | 42 | End of no passing by vehicles over 3.5 metric tons |

![Dataset label repartition][image1]

### Design and Test a Model Architecture

#### 1. Describe how you preprocessed the image data. What techniques were chosen and why did you choose these techniques? Consider including images showing the output of each preprocessing technique. Pre-processing refers to techniques such as converting to grayscale, normalization, etc. (OPTIONAL: As described in the "Stand Out Suggestions" part of the rubric, if you generated additional data for training, describe why you decided to generate additional data, how you generated the data, and provide example images of the additional data. Then describe the characteristics of the augmented training set like number of images in the set, number of images for each class, etc.)

##### 1.1. Initial dataset
Here is an example of each label before any processing:
![Dataset label examples][image2]

##### 1.2. Augmented dataset
Since some traffic signs were ill represented, augmentation was done so as not to bias the model with lack of some signs.
A combination of random translation, perspective change and brightness adjust was used.
![Dataset augmented examples][image3]

![Dataset augmented label repartition][image4]

##### 1.3. Grayscale dataset
Since color is not as important as for traffic light claissification, I decided to convert the images to grayscale to reduce the number of features and simplify classification.
If this model was to be enhanced, color could be added to easily rule out range of signs.
![Dataset grayscale examples][image5]

##### 1.4. Shuffle dataset
Since images in the dataset are sorted by label,the dataset was shuffled so that no drift occurs after many successions of the same sign during the learning
![Dataset shuffled label repartition][image6]

##### 1.4. Normalize dataset
To speed up learning and reach faster convergence, normalize dataset to obtain a mean close to 0 
 * Non-normalized mean: 80.038
 * Normalized mean: -0.375

#### 2. Describe what your final model architecture looks like including model type, layers, layer sizes, connectivity, etc.) Consider including a diagram and/or table describing the final model.

I used the standard LeNet model with 43 output classes:
 * Initial data 32x32x1
 * Convolution 5x5->6: 28x28x6 (32-5+1)
 * ReLU
 * Pooling 14x14: 14x14x6 (28/2)
 * Convolution 5x5->16: 10x10x16 (14-5+1)
 * ReLU
 * Pooling 5x5: 5x5x16 (28/2) 
 * Flatten: 400 (5x5x16)
 * Fully connected layer: 120
 * ReLU
 * Fully connected layer: 84
 * ReLU
 * Fully connected layer: 43
![LeNet 43 classes][image7]

#### 3. Describe how you trained your model. The discussion can include the type of optimizer, the batch size, number of epochs and any hyperparameters such as learning rate.

After a few iterations, the following hyperparameters were use:
 * AdamOptimizer
 * 64 batch size 
 * 706 epochs
 * 0.0005 learning rate

#### 4. Describe the approach taken for finding a solution and getting the validation set accuracy to be at least 0.93. Include in the discussion the results on the training, validation and test sets and where in the code these were calculated. Your approach may have been an iterative process, in which case, outline the steps you took to get to the final solution and why you chose those steps. Perhaps your solution involved an already well known implementation or architecture. In this case, discuss why you think the architecture is suitable for the current problem.

My final model results were:
* training set accuracy of 100%
* validation set accuracy of 94.5%
* test set accuracy of 93.7%

Here are the iterations to reach the final solution (accuracy is validation accuracy):
* Implement the standard LeNet model as a baseline to work on
  * 5 epochs, 128 batch size, 0.1 learning rate
  * 5% accuracy
* Hyperparameters are obviusly an issue
  * Change batch size and epochs: No effect
  * Divide learning rate by 10 multiple times and train 5 times (not 5 epochs, 5 different models):
    * 0.1 -> 5.4 (Does not converge)
    * 0.01 -> [87;93]
    * 0.001 -> [81;92]
    * 0.0001 -> [59;87
    * 0.00001 -> 9 (Too slow to converge)
  * 0.01 seems like a good learning rate but what if we give the model more time to converge for smaller learning rates?
    * Adjusted epochs to 60 and tried learning rates between 0.001 and 0.0001 (.0005/.0007/.0003)
    * 0.0005 -> [92;93] Best result yielded, other result were similar or below
  * Augmented data for ill represented labels
    * 93.9 accuracy
  * Noticed that 60 epochs * 128 images batch does not process all training data
    * As per the following articles, batch size should not be too high (64 recommended) and the more epochs the better
      * https://medium.com/mini-distill/effect-of-batch-size-on-training-dynamics-21c14f7a716e
      * https://www.ijstr.org/final-print/jun2020/Significance-Of-Epochs-On-Training-A-Neural-Network.pdf
    * 64 batch size, 706 epochs (images count divided by batch size), 0.0005 learning rate
    * 94.5 accuracy

The LeNet architecture was chosen because traffic signs are all about shape, just like numbers

The model training accuracy is 100%, but the validation and test accuracy are not too far off, around 94%.
This gives confidence that we do not have an over fitting issue.

### Test a Model on New Images

#### 1. Choose five German traffic signs found on the web and provide them in the report. For each image, discuss what quality or qualities might be difficult to classify.

Here are five German traffic signs found on the web:
![German dataset][image8]
 * No entry: Might be difficult to classify because of the graffiti
 * No entry: No particular difficulty for classifying, just testing
 * Road work: Might be difficult to classify because of wear
 * Slippery raod: Might be difficult to classify because tilted
 * Stop: Might be difficult to classify because of the holes

#### 2. Discuss the model's predictions on these new traffic signs and compare the results to predicting on the test set. At a minimum, discuss what the predictions were, the accuracy on these new predictions, and compare the accuracy to the accuracy on the test set (OPTIONAL: Discuss the results in more detail as described in the "Stand Out Suggestions" part of the rubric).

Here are the results of the prediction:
![German dataset prediction][image9]

The model was able to correctly guess 4 of the 5 traffic signs
The accuracy is 80%.
Given that this dataset is quite small, this accuracy is in line with the results obtained from validation and test.

#### 3. Describe how certain the model is when predicting on each of the five new images by looking at the softmax probabilities for each prediction. Provide the top 5 softmax probabilities for each image along with the sign type of each probability. (OPTIONAL: as described in the "Stand Out Suggestions" part of the rubric, visualizations can also be provided such as bar charts)

The model is quite convinced for each guess it has taken for each images from the German dataset:
![German dataset certainty][image10]

### (Optional) Visualizing the Neural Network (See Step 4 of the Ipython notebook for more details)
#### 1. Discuss the visual output of your trained network's feature maps. What characteristics did the neural network use to make classifications?
Not performed

