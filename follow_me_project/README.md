# Project: Follow Me

## Required Steps for a Passing Submission:

1. Clone the repo [here](https://github.com/udacity/RoboND-DeepLearning-Project.git) 
2. Fill out the TODO's
3. Optimize your network and hyper-parameters
4. Train your network and achieve an accuracy of  40% (0.40) using the intersection over union metric
which is `final_grade_score` at the bottom of your notebook
5. Make a brief writeup report summarizing why you made the choices you did in building the network

## [Rubric](https://review.udacity.com/#!/rubrics/1155/view) Points

## Submission:

1. Your `model_training.ipynb` notebook that you have filled out
2. An HTML version of your `model_training.ipynb` notebook
3. Writeup report
4. Your model and weights file in `.h5` file format

---

### [model_training.ipnyb Here](https://github.com/vuvuzella/udacity-robond-term1/blob/master/follow_me_project/RoboND-DeepLearning-Project/code/model_training.ipynb)
### [model_training.html Here](./RoboND-DeepLearning-Project/code/model_training.html)
### [Model and weights config Here](https://github.com/vuvuzella/udacity-robond-term1/blob/master/follow_me_project/RoboND-DeepLearning-Project/data/weights)

## Writeup / README

## Network Architecture
 - A Fully Convolutional Network architecture was used 
 to train the model.

 - Having deeper networks extracts more features from an input image.
  By using convolutional networks, we also lessen the occurrence of overfitting,
  Compared to using Fully Connected Networks.

 - The addition of 1x1 convolutional networks adds more depth, without the
 computational burden of a normal convolutional network.

 The following describes the layers of the network:

   1. **Layers**

      - **Encoder Block**
        - Separable Convolution Layer
          - One purpose of convolutional networks is to increase the depth of the network
           without the increasing the number of parameter weights at an exploding rate (which is computationally costly.)
           However, Separable Convolutions extends this by using 1x1 convolusion instead of relying on the n-outputs of the usual convolutional layer.
           This combination drastically reduces weight parameters and at the same time, gaining the same depth in the output feature maps.
        - The implementation also includes batch normalization is also done for after the separable convolution layer,
         which keeps mean results of the convolution close to 0, and standard deviation close to 1. This means we could set learning rates higher as
          well as rendering our network to be resilient to weights that were initialized with high deviations.

      - **1x1 Convolution Layer**
        - A 1x1 convolution layer is used to control the spatial information/depth of inputs/feature maps
         without incurring the computation cost of a convolutional network that has
          a bigger than 1x1 kernel/filter size. It adds/subtracts depth to your network.
        - 1x1 convolution layer can be added after a previous convolution has been calculated,
          either to add or subtract depth (using `n` number of filters).
        - Usage of 1x1 convolutions are ideal for networks that would be trained on low or medium-spec'ed
         computers.

      - **Decoder Block**
        - Bilinear Upsampling
          - The inputs are upscaled as part of the `decoding` or `deconvolutional` phase of the Fully Convolutional Network.
          - The inputs are upscaled by a factor of 2
        - Skip Connections
          - Skip connections are used to add back in the parts of the original image that were lost during the convolution.
           This is done by adding in a non-adjacent layer of the same dimension
        - Separable Convolution of a specific kernel size 

      - **Fully Connected Network**
        - The downside of having a fully connected layer is that the growth of your parameters is exponential. 
         This means that as the networks gets deeper with Fully connected layers, the number of parameters to fit also increases exponentially.
          This would result into costly computations and perils of overfitting the model.
          Moreover, Fully connected layers squash inputs into 2D arrays/tensors as outputs, so for 3D inputs like images,
          Spatial informan is lost in the output feature map of a Fully connected layer.
          Because of this, Fully connected layers should be appended only after the final layers of a network 
          (in the case of a Fully Convolutional Network, Fully connected layers is connected after the last decoding block)
          where the output will be the result is the predicted value.

   2. **Architecture**

      - Encoder blocks - 4 Encoder blocks that consist of a separable convolution
       layer with bath normalization applied.
      The first encoder block has a depth of 16. The second block has 32.
      The third block has 64 and the 4th block has 128. Each block has 
      a stride of 2, which halves the size of the image for each block. 
      The image dimensions are 10x10 by the time it inters the 1x1 convolution
       layer. The filter size for each encoder block is 3.

      - 1x1 Convolution - 1 convolution block with a filter size of 1, 
      stride of 1 and a depth of 256. 
        - Suffer less from overfitting because of the size of the 
        kernel (1x1)
       
      - Decoder - 4 Decoder blocks. Each Decoder block has an upsampling function 
      that upscales the output from the previous layers. The upsampling
       factor is by 2, doubling the dimension of the input image.
        Each block has a skip connection to an encoder block 
       that has the same size after upsampling. After the skip connection 
       (concatenation), the image in each block then undergoes 2 separable 
       convolution and batch normalization layers with a kernel size of 3 and
       stride of 1. Each decoder block, from the 1x1 convolution block, has the 
       filter size of 3 each, and whose depth sizes are 128, 64, 32 and 16
        respectively.

      - The final decoder layer is then connected to a fully connected layer
       whose output is 3, which then uses a softmax function to produce 
       the probability for each class/output.

      - All separable convolutions have Relu as it's activation function.

      - All layers uses 'same' padding.

Here is a graph of the network architecture:
![Network](./RoboND-DeepLearning-Project/code/model.png)

   3. **Reasons for encoding/decoding images, when it should be used, why it's useful, and any problems that may arise.** 
      - **Encoding** images is useful to effectively downsample an input image. As the image goes through each encoding
      blocks, the image dimension is reduced. Also, only the neuron that has the highest value in its activation function output (max pooling)
      gets retained at the end of the encoding chain resulting in a greatly reduced image/tensor with only the remaining activated neurons intact.
      By encoding images, we have narrowed down to a specific visual feature.
      - **Decoding** images is done to reconstruct the image back to its original dimension, with the added information on the narrowed down visual feature,
      to be evaluated by a final fully connected layer for evaluation (probabilities for each class outputs).
      The Decoding layers also gain from backpropagation phase during training, contributing to the final output/prediction.
      - **Skip Connections** To recover the lost spatial information during the encoding phase, the activation result from previous layers are added in to the decoded feature map to "add back in" the data that were thrown away during encoding phase. This results in better segmentations.
      - **Problems** 
        - Small objects that might be of interest might be lost during encoding
        - Data inputs need to be structured (might require data augmentation)

## Hyper Parameter Tuning

  1. Epoch
     - The starting epoch was 10 just to see how the learning rate and the 
     overall initial network architecture performs. When the network was hitting
      around 29-33% IoU, the epoch was then increased using. The final epoch
      is 50.

  2. Learning Rate
     - The learning rate started from 0.001 and was gradually increased as 
      when minimization of loss was quite slow for the given epoch. Finally,
      the learning rate was set to 0.01 for the final epoch, whcih achieved
       the IoU > 40% goal

  3. Batch Size
     - The initial batch size was at 64, when the network was simple, but eventually,
     I had to lower down the batch size as my network got bigger and more deeper to
      avoid out of memory error. Eventually I settled with 25.

  4. Steps per Epoch
     - I have added some additional data, but not much. My total training data
      is 4300. Given my batch size as 25, then each epoch would have 172 steps.
       Hence my steps per epoch value is 172.
  
  5. Validation Steps
     - This was left untouched, even if there were added validation data.
      The value is 50.
  
  6. Workers
     - The number of threads that were allocated for training was left at 
     default value of 2.

## Result

1. Intersection over Union is used to measure the performance of the trained model.
The Final IoU score is calculated as Final IoU divided by normalized true positive.

       Final IoU: 0.55418
       Normalized True Positive: 0.75744
       Final Score: 0.41976 or 41%

Some pictures of Hero detection:

1. Following target

   ![Following target](./writeup_files/eval_following_target.png)

2. Patrol without target

   ![Patrol without target](./writeup_files/patrol_no_target.png)

3. Patrol with target

   ![Patrol without target](./writeup_files/patrol_with_target.png)

4. Video

   [![Follow Me Project](http://img.youtube.com/vi/6RUyYfu2jHU/0.jpg)](https://www.youtube.com/watch?v=6RUyYfu2jHU)

## Limitations
1. Would the trained model and data work well for following another object
 (dog, car, cat, etc.) instead of a human and if not, what changes would be required?
 - The resulting model would not work well if we would want to follow a different object.
 - To be able to follow a different another object, we must train the model using training and validation image data that contains
   the object we need to detect. These data must be properly labeled using image masks to serve as the ground truths of our
   data in general. We then use these new sets of data train/re-train the model, adjust the parameters if necessary

## Future Enchancements

1. **Do Data Augmentation**
2. **Delete training data that may not contribute to the training**
3. **Use other optimizing techniques (Nadam instead of Adam)**

