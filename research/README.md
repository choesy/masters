# Algorithm Description

1. **Objective**: The algorithm's objective is to detect anomalies in one group of measured features compared to another group of measured features.

2. **Data Sampling**:
   - Each sample in the group consists of selected features from a 5-second window of accelerometer signal data collected from a Caretronic wristband.
   - The dataset for training and testing is evenly distributed among all accelerometer recordings for various individuals.
   - Training data uses recordings where individuals wore the Caretronic wristband on their left hand, while testing data includes recordings where individuals wore it on their right hand.

3. **Data Normalization**: All data is normalized to a range between 0 and 1.

4. **Distance Metrics**: The algorithm employs various distance metrics, including:
   - Euclidean distance
   - Cosine distance
   - Mahalanobis distance
   - Jensen-Shannon divergence
   - Maximum mean discrepancy (MMD)
   - Wasserstein distance

5. **Personalization**: The algorithm is personalized, meaning it calculates a unique distance threshold (threshold) for each individual. Data exceeding this threshold is classified as anomalous.

6. **Classification Problem**:
   - The problem is treated as a classification task.
   - Sessions 1 and 2 in the training dataset are labeled as "0" (negative class), while sessions 5 and 6 are labeled as "1" (positive class).
   - In the test dataset, sessions 3 and 4 are labeled as "0," and sessions 7 and 8 are labeled as "1."

7. **Reference Data**: For each individual, data from the negative class in the training set serves as reference or baseline data for comparison with test data.

8. **Threshold Calculation**:
   - To determine the distance threshold between the reference (negative) and positive data groups, the algorithm considers the internal variability of features in the reference data for each of the 17 individuals.
   - The reference data (sessions 1 and 2) in the training set is split into two subsets, with session 1 as the first subset and session 2 as the second.
   - The calculated distance between these subsets becomes the threshold, which can then be scaled by a constant to reduce false positives.

9. **Feature Selection Evaluation**: The algorithm evaluates the performance of different feature selection methods and distance metrics in four different combinations. These include features obtained from accelerometer data along three axes and the use of methods like RFECV and LASSO for feature selection.

10. **Visualization**: Data visualization is done using the t-distributed stochastic neighbor embedding (TSNE) method to reduce dimensionality and cluster data in a 2D space.

This algorithm aims to identify anomalies in accelerometer data collected from Caretronic wristbands and offers personalized thresholding for improved detection accuracy.