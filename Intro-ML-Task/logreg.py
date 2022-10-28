import pandas as pd
import numpy as np

train_df = pd.read_csv("/home/sirabas/MNIST/train.csv")
test_df = pd.read_csv("/home/sirabas/MNIST/test.csv")
X_train = train_df.drop('label',axis=1)
y_train = train_df['label']
X_test= test_df



from sklearn.neighbors import KNeighborsClassifier

knn = KNeighborsClassifier(n_neighbors=3)

knn.fit(X_train, y_train)

y_predict=knn.predict(X_test)

image_id = pd.Series(range(1,28001),name='ImageId')
y_preds = pd.Series(y_predict,name = 'Label')
pred = pd.concat([image_id,y_preds])
pred.to_csv('submission.csv',index=False)
