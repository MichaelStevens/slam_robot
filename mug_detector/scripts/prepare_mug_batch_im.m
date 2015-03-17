% ------------------------------------------------------------------------
function images = prepare_mug_batch_im(images_raw,pixel_mean,batch_size)
% ------------------------------------------------------------------------


IMAGE_DIM = 227;
indices = [0 IMAGE_DIM] + 1;

num_images = length(images_raw);
images = zeros(IMAGE_DIM,IMAGE_DIM,3,batch_size,'single');

parfor i=1:num_images
    % read file
    %fprintf('%c Preparing %s\n',13,image_files{i});
    try
        im = images_raw{i};
        % resize to fixed input size
        im = single(im);
        im = imresize(im, [IMAGE_DIM IMAGE_DIM], 'bilinear');
       
        % permute from RGB to BGR (IMAGE_MEAN is already BGR)
        im = im(:,:,[3 2 1]);
        im(:, :, 1) = im(:, :, 1) - pixel_mean(1);
        im(:, :, 2) = im(:, :, 2) - pixel_mean(2);
        im(:, :, 3) = im(:, :, 3) - pixel_mean(3);
        
        % permute so the width is the fastest dimension
        images(:,:,:,i) = permute(im,[2 1 3]);
    catch
        warning('Problems with file',images_raw{i});
    end
end