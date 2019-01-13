import rosbag_pandas
import numpy as np

from donkey_keras.utils import image_deserialize


class DatastoreException(Exception):
    pass


class Datastore:
    def __init__(self, bag_path, bag_topics, sample_freq_hz=30.0):
        self.df = None
        self.bag_path = bag_path
        self.bag_topics = bag_topics
        self.sample_freq_hz = sample_freq_hz

    def update_df(self, image_keys):
        sample_freq = '{:0.4f}S'.format(1.0 / self.sample_freq_hz)
        df = rosbag_pandas.bag_to_dataframe(bag_name=self.bag_path,
                                            include=self.bag_topics)
        # Bin and resample image and drive command frames in order to gain
        # cleanly labeled data. Sample frequency should be less than
        # or equal to the most infrequent data sequence (e.g. images are
        # typically collected at ~30hz)
        self.df = df.resample(sample_freq).last().dropna()

    def get_records(self, image_keys, record_transform=None, shuffle=True, df=None):
        """
        Parameters
        ----------
        image_keys : array of strings
            The columns containing image data in need of deserialization
        record_transform : function
            The mapping function should handle records in dict format
        shuffle : bool
            Shuffle records
        df : numpy Dataframe
            If df is specified, the generator will use the records specified in that DataFrame. If None,
            the internal DataFrame will be used by calling get_df()

        Returns
        -------
        A dict with keys mapping to the specified keys, and values lists of size batch_size.
        """
        while True:
            for _ in self.df.iterrows():
                if shuffle:
                    record_dict = df.sample(n=1).to_dict(orient='record')[0]

                for key in image_keys:
                    record_dict[key] = image_deserialize(record_dict[key])

                if record_transform:
                    record_dict = record_transform(record_dict)

                yield record_dict
        
    def get_batch_gen(self, X_keys, Y_keys, batch_size=128, record_transform=None, df=None):
        """
        Returns batches of records.

        Additionally, each record in a batch is split up into a dict with inputs:list of values. By specifying keys as a subset of the inputs, you can filter out unnecessary data.

        Parameters
        ----------
        keys : list of strings
            List of keys to filter out. If None, all inputs are included.
        batch_size : int
            The number of records in one batch.

        Returns
        -------
        A dict with keys mapping to the specified keys, and values lists of size batch_size.
        """
        record_gen = self.get_records(X_keys, record_transform=record_transform, df=df)

        while True:
            record_list = [ next(record_gen) for _ in range(batch_size) ]

            batch_arrays = {}
            for k in (X_keys + Y_keys):
                arr = np.array([r[k] for r in record_list])
                batch_arrays[k] = arr
            yield batch_arrays
        
    def get_train_gen(self, X_keys, Y_keys,
                      batch_size=128,
                      record_transform=None,
                      df=None):
        """
        Returns a training/validation set.

        The records are always shuffled.

        Parameters
        ----------
        X_keys : list of strings
            List of the feature(s) to use. Must be included in Tub.inputs.
        Y_keys : list of strings
            List of the label(s) to use. Must be included in Tub.inputs.

        Returns
        -------
        A tuple (X, Y), where X is a two dimensional array ( len(X_keys) x batch_size ) and Y is a two dimensional array ( len(Y_keys) x batch_size ).
        """
        batch_gen = self.get_batch_gen(X_keys, Y_keys,
                                       batch_size=batch_size,
                                       record_transform=record_transform,
                                       df=df)

        while True:
            batch = next(batch_gen)
            X = [batch[k] for k in X_keys]
            Y = [batch[k] for k in Y_keys]
            yield X, Y

    def get_train_val_gen(self, X_keys, Y_keys, batch_size=128, train_frac=.8,
                      train_record_transform=None, val_record_transform=None):
        """
        Create generators for training and validation set.

        Parameters
        ----------
        train_frac : float
            Training/validation set split.
        train_record_transform : function
            Transform function for the training set. Used internally by Tub.get_record_gen().
        val_record_transform : function
            Transform  function for the validation set. Used internally by Tub.get_record_gen().

        Returns
        -------
        A tuple (train_gen, val_gen), where where train_gen is the training set generator, and
        val_gen the validation set generator.
        """
        if self.df is None:
            self.update_df(X_keys)

        train_df = self.df.sample(frac=train_frac, random_state=200)
        val_df = self.df.drop(train_df.index)

        train_gen = self.get_train_gen(X_keys=X_keys, Y_keys=Y_keys, batch_size=batch_size,
                                       record_transform=train_record_transform, df=train_df)

        val_gen = self.get_train_gen(X_keys=X_keys, Y_keys=Y_keys, batch_size=batch_size,
                                     record_transform=val_record_transform, df=val_df)

        return train_gen, val_gen
