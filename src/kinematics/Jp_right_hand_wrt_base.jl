function Jp_right_hand_wrt_base_helper!(p_output1, var1) 
  t2673 = sin(var1[3+1])
  t2685 = cos(var1[26+1])
  t2686 = -1.0 * t2685
  t2702 = 1.0 + t2686
  t2705 = sin(var1[26+1])
  t2729 = cos(var1[3+1])
  t2710 = cos(var1[5+1])
  t2722 = sin(var1[4+1])
  t2730 = sin(var1[5+1])
  t2604 = cos(var1[27+1])
  t2612 = -1.0 * t2604
  t2614 = 1.0 + t2612
  t2654 = sin(var1[27+1])
  t2724 = -1.0 * t2710*t2673*t2722
  t2735 = t2729*t2730
  t2738 = t2724 + t2735
  t2754 = -1.0 * t2729*t2710
  t2755 = -1.0 * t2673*t2722*t2730
  t2756 = t2754 + t2755
  t2839 = cos(var1[28+1])
  t2840 = -1.0 * t2839
  t2846 = 1.0 + t2840
  t2849 = sin(var1[28+1])
  t2570 = cos(var1[4+1])
  t2791 = -1.0 * t2705*t2738
  t2793 = t2685*t2756
  t2799 = t2791 + t2793
  t2805 = t2685*t2738
  t2822 = t2705*t2756
  t2823 = t2805 + t2822
  t2848 = -0.051978134642000004*t2846
  t2890 = 0.05226439969100001*t2846
  t2855 = 0.49726168403800003*t2846
  t2941 = 0.073913*t2849
  t2873 = 0.994522*t2570*t2654*t2673
  t2876 = -0.103955395616*t2614*t2799
  t2877 = -0.9890740084840001*t2614
  t2878 = 1.0 + t2877
  t2879 = t2878*t2823
  t2881 = t2873 + t2876 + t2879
  t2938 = -0.703234*t2849
  t2916 = 0.104528*t2570*t2654*t2673
  t2917 = -0.010926102783999999*t2614
  t2918 = 1.0 + t2917
  t2919 = t2918*t2799
  t2923 = -0.103955395616*t2614*t2823
  t2925 = t2916 + t2919 + t2923
  t2948 = -1.0000001112680001*t2614
  t2949 = 1.0 + t2948
  t2950 = -1.0 * t2949*t2570*t2673
  t2952 = 0.104528*t2654*t2799
  t2954 = 0.994522*t2654*t2823
  t2955 = t2950 + t2952 + t2954
  t2961 = cos(var1[29+1])
  t2963 = -1.0 * t2961
  t2966 = 1.0 + t2963
  t2969 = sin(var1[29+1])
  t2850 = -0.707107*t2849
  t2980 = -0.49726168403800003*t2846
  t2867 = -0.073913*t2849
  t3016 = 0.051978134642000004*t2846
  t2901 = 0.707107*t2849
  t2986 = -0.05226439969100001*t2846
  t2891 = 0.703234*t2849
  t2983 = t2980 + t2941
  t2984 = t2983*t2881
  t2988 = t2986 + t2938
  t2993 = t2988*t2925
  t2995 = -0.500001190325*t2846
  t2999 = 1.0 + t2995
  t3000 = t2999*t2955
  t3003 = t2984 + t2993 + t3000
  t3013 = -0.5054634410180001*t2846
  t3014 = 1.0 + t3013
  t3015 = t3014*t2881
  t3017 = t3016 + t2850
  t3021 = t3017*t2925
  t3022 = t2980 + t2867
  t3023 = t3022*t2955
  t3024 = t3015 + t3021 + t3023
  t3031 = t3016 + t2901
  t3036 = t3031*t2881
  t3037 = -0.9945383682050002*t2846
  t3038 = 1.0 + t3037
  t3039 = t3038*t2925
  t3041 = t2986 + t2891
  t3043 = t3041*t2955
  t3044 = t3036 + t3039 + t3043
  t3046 = -0.104528*t2969*t3003
  t3047 = -0.103955395616*t2966*t3024
  t3048 = -0.010926102783999999*t2966
  t3052 = 1.0 + t3048
  t3053 = t3052*t3044
  t3054 = t3046 + t3047 + t3053
  t3057 = -0.994522*t2969*t3003
  t3059 = -0.9890740084840001*t2966
  t3060 = 1.0 + t3059
  t3061 = t3060*t3024
  t3064 = -0.103955395616*t2966*t3044
  t3065 = t3057 + t3061 + t3064
  t3069 = -1.0000001112680001*t2966
  t3070 = 1.0 + t3069
  t3071 = t3070*t3003
  t3073 = 0.994522*t2969*t3024
  t3074 = 0.104528*t2969*t3044
  t3075 = t3071 + t3073 + t3074
  t2643 = -0.056500534356700764*t2614
  t2664 = 0.3852490428658858*t2654
  t2667 = t2643 + t2664
  t2704 = 0.4*t2702
  t2706 = -0.12*t2705
  t2707 = t2704 + t2706
  t2745 = -0.12*t2702
  t2748 = -0.4*t2705
  t2749 = t2745 + t2748
  t2774 = 1.1345904784751044e-7*var1[27+1]
  t2779 = 0.0402693119526853*t2614
  t2782 = 0.0059058871981009595*t2654
  t2788 = t2774 + t2779 + t2782
  t3093 = t2729*t2710*t2722
  t3094 = t2673*t2730
  t3095 = t3093 + t3094
  t3098 = -1.0 * t2710*t2673
  t3099 = t2729*t2722*t2730
  t3101 = t3098 + t3099
  t2801 = -1.1924972351948546e-8*var1[27+1]
  t2802 = 0.3831386486090665*t2614
  t2803 = 0.05619101817723254*t2654
  t2804 = t2801 + t2802 + t2803
  t2830 = 4.0332087336819504e-7*var1[28+1]
  t2847 = 0.0958179942122405*t2846
  t2851 = t2848 + t2850
  t2852 = -0.23105307644*t2851
  t2868 = t2855 + t2867
  t2869 = 0.164374659834*t2868
  t2872 = t2830 + t2847 + t2852 + t2869
  t3103 = -1.0 * t2705*t3095
  t3105 = t2685*t3101
  t3106 = t3103 + t3105
  t3110 = t2685*t3095
  t3111 = t2705*t3101
  t3112 = t3110 + t3111
  t2884 = 4.239080549754904e-8*var1[28+1]
  t2885 = -0.22979114961138278*t2846
  t2892 = t2890 + t2891
  t2896 = 0.164374659834*t2892
  t2907 = t2848 + t2901
  t2908 = 0.189564637987*t2907
  t2909 = t2884 + t2885 + t2896 + t2908
  t2936 = -4.05542127947119e-7*var1[28+1]
  t2937 = 0.08218752557626696*t2846
  t2939 = t2890 + t2938
  t2940 = -0.23105307644*t2939
  t2942 = t2855 + t2941
  t2945 = 0.189564637987*t2942
  t2946 = t2936 + t2937 + t2940 + t2945
  t2968 = 0.19098732144477495*t2966
  t2974 = 0.13776101532839094*t2969
  t2975 = t2968 + t2974
  t3114 = -0.994522*t2729*t2570*t2654
  t3115 = -0.103955395616*t2614*t3106
  t3116 = t2878*t3112
  t3117 = t3114 + t3115 + t3116
  t3121 = -0.104528*t2729*t2570*t2654
  t3122 = t2918*t3106
  t3126 = -0.103955395616*t2614*t3112
  t3127 = t3121 + t3122 + t3126
  t3132 = t2949*t2729*t2570
  t3133 = 0.104528*t2654*t3106
  t3134 = 0.994522*t2654*t3112
  t3135 = t3132 + t3133 + t3134
  t3007 = 5.06291820800569e-8*var1[29+1]
  t3008 = 0.13700636048642204*t2966
  t3011 = -0.18994107176353728*t2969
  t3012 = t3007 + t3008 + t3011
  t3027 = -4.817066759205414e-7*var1[29+1]
  t3028 = 0.014399883410246048*t2966
  t3029 = -0.019963520514678434*t2969
  t3030 = t3027 + t3028 + t3029
  t3139 = t2983*t3117
  t3140 = t2988*t3127
  t3141 = t2999*t3135
  t3142 = t3139 + t3140 + t3141
  t3146 = t3014*t3117
  t3147 = t3017*t3127
  t3150 = t3022*t3135
  t3151 = t3146 + t3147 + t3150
  t3153 = t3031*t3117
  t3157 = t3038*t3127
  t3159 = t3041*t3135
  t3160 = t3153 + t3157 + t3159
  t3165 = -0.104528*t2969*t3142
  t3167 = -0.103955395616*t2966*t3151
  t3168 = t3052*t3160
  t3169 = t3165 + t3167 + t3168
  t3171 = -0.994522*t2969*t3142
  t3172 = t3060*t3151
  t3173 = -0.103955395616*t2966*t3160
  t3174 = t3171 + t3172 + t3173
  t3176 = t3070*t3142
  t3177 = 0.994522*t2969*t3151
  t3179 = 0.104528*t2969*t3160
  t3180 = t3176 + t3177 + t3179
  t3198 = -1.0 * t2729*t2570*t2710*t2705
  t3199 = t2685*t2729*t2570*t2730
  t3202 = t3198 + t3199
  t3204 = t2685*t2729*t2570*t2710
  t3205 = t2729*t2570*t2705*t2730
  t3206 = t3204 + t3205
  t3209 = 0.994522*t2729*t2654*t2722
  t3210 = -0.103955395616*t2614*t3202
  t3211 = t2878*t3206
  t3212 = t3209 + t3210 + t3211
  t3214 = 0.104528*t2729*t2654*t2722
  t3215 = t2918*t3202
  t3216 = -0.103955395616*t2614*t3206
  t3217 = t3214 + t3215 + t3216
  t3221 = -1.0 * t2949*t2729*t2722
  t3222 = 0.104528*t2654*t3202
  t3224 = 0.994522*t2654*t3206
  t3227 = t3221 + t3222 + t3224
  t3229 = t2983*t3212
  t3230 = t2988*t3217
  t3231 = t2999*t3227
  t3232 = t3229 + t3230 + t3231
  t3234 = t3014*t3212
  t3235 = t3017*t3217
  t3236 = t3022*t3227
  t3237 = t3234 + t3235 + t3236
  t3240 = t3031*t3212
  t3243 = t3038*t3217
  t3244 = t3041*t3227
  t3245 = t3240 + t3243 + t3244
  t3247 = -0.104528*t2969*t3232
  t3248 = -0.103955395616*t2966*t3237
  t3249 = t3052*t3245
  t3250 = t3247 + t3248 + t3249
  t3254 = -0.994522*t2969*t3232
  t3255 = t3060*t3237
  t3256 = -0.103955395616*t2966*t3245
  t3258 = t3254 + t3255 + t3256
  t3262 = t3070*t3232
  t3263 = 0.994522*t2969*t3237
  t3264 = 0.104528*t2969*t3245
  t3266 = t3262 + t3263 + t3264
  t3286 = -1.0 * t2570*t2710*t2705*t2673
  t3288 = t2685*t2570*t2673*t2730
  t3292 = t3286 + t3288
  t3294 = t2685*t2570*t2710*t2673
  t3297 = t2570*t2705*t2673*t2730
  t3298 = t3294 + t3297
  t3300 = 0.994522*t2654*t2673*t2722
  t3301 = -0.103955395616*t2614*t3292
  t3302 = t2878*t3298
  t3303 = t3300 + t3301 + t3302
  t3305 = 0.104528*t2654*t2673*t2722
  t3306 = t2918*t3292
  t3307 = -0.103955395616*t2614*t3298
  t3308 = t3305 + t3306 + t3307
  t3310 = -1.0 * t2949*t2673*t2722
  t3311 = 0.104528*t2654*t3292
  t3313 = 0.994522*t2654*t3298
  t3314 = t3310 + t3311 + t3313
  t3317 = t2983*t3303
  t3318 = t2988*t3308
  t3319 = t2999*t3314
  t3322 = t3317 + t3318 + t3319
  t3324 = t3014*t3303
  t3325 = t3017*t3308
  t3326 = t3022*t3314
  t3328 = t3324 + t3325 + t3326
  t3331 = t3031*t3303
  t3332 = t3038*t3308
  t3333 = t3041*t3314
  t3334 = t3331 + t3332 + t3333
  t3336 = -0.104528*t2969*t3322
  t3337 = -0.103955395616*t2966*t3328
  t3338 = t3052*t3334
  t3340 = t3336 + t3337 + t3338
  t3342 = -0.994522*t2969*t3322
  t3343 = t3060*t3328
  t3344 = -0.103955395616*t2966*t3334
  t3345 = t3342 + t3343 + t3344
  t3347 = t3070*t3322
  t3348 = 0.994522*t2969*t3328
  t3349 = 0.104528*t2969*t3334
  t3350 = t3347 + t3348 + t3349
  t3370 = t2710*t2705*t2722
  t3371 = -1.0 * t2685*t2722*t2730
  t3372 = t3370 + t3371
  t3374 = -1.0 * t2685*t2710*t2722
  t3375 = -1.0 * t2705*t2722*t2730
  t3376 = t3374 + t3375
  t3378 = 0.994522*t2570*t2654
  t3379 = -0.103955395616*t2614*t3372
  t3380 = t2878*t3376
  t3381 = t3378 + t3379 + t3380
  t3383 = 0.104528*t2570*t2654
  t3384 = t2918*t3372
  t3385 = -0.103955395616*t2614*t3376
  t3386 = t3383 + t3384 + t3385
  t3388 = -1.0 * t2949*t2570
  t3389 = 0.104528*t2654*t3372
  t3392 = 0.994522*t2654*t3376
  t3393 = t3388 + t3389 + t3392
  t3395 = t2983*t3381
  t3396 = t2988*t3386
  t3397 = t2999*t3393
  t3399 = t3395 + t3396 + t3397
  t3401 = t3014*t3381
  t3402 = t3017*t3386
  t3403 = t3022*t3393
  t3404 = t3401 + t3402 + t3403
  t3406 = t3031*t3381
  t3407 = t3038*t3386
  t3408 = t3041*t3393
  t3409 = t3406 + t3407 + t3408
  t3415 = -0.104528*t2969*t3399
  t3416 = -0.103955395616*t2966*t3404
  t3417 = t3052*t3409
  t3419 = t3415 + t3416 + t3417
  t3423 = -0.994522*t2969*t3399
  t3426 = t3060*t3404
  t3427 = -0.103955395616*t2966*t3409
  t3428 = t3423 + t3426 + t3427
  t3430 = t3070*t3399
  t3431 = 0.994522*t2969*t3404
  t3432 = 0.104528*t2969*t3409
  t3433 = t3430 + t3431 + t3432
  t3446 = t2710*t2673
  t3447 = -1.0 * t2729*t2722*t2730
  t3448 = t3446 + t3447
  t3450 = t2705*t3095
  t3451 = t2685*t3448
  t3452 = t3450 + t3451
  t3455 = -1.0 * t2705*t3448
  t3456 = t3110 + t3455
  t3459 = -0.103955395616*t2614*t3452
  t3460 = t2918*t3456
  t3461 = t3459 + t3460
  t3463 = t2878*t3452
  t3464 = -0.103955395616*t2614*t3456
  t3465 = t3463 + t3464
  t3468 = 0.994522*t2654*t3452
  t3469 = 0.104528*t2654*t3456
  t3470 = t3468 + t3469
  t3472 = t2988*t3461
  t3473 = t2983*t3465
  t3474 = t2999*t3470
  t3475 = t3472 + t3473 + t3474
  t3477 = t3017*t3461
  t3480 = t3014*t3465
  t3482 = t3022*t3470
  t3484 = t3477 + t3480 + t3482
  t3489 = t3038*t3461
  t3490 = t3031*t3465
  t3491 = t3041*t3470
  t3492 = t3489 + t3490 + t3491
  t3494 = -0.104528*t2969*t3475
  t3495 = -0.103955395616*t2966*t3484
  t3496 = t3052*t3492
  t3498 = t3494 + t3495 + t3496
  t3500 = -0.994522*t2969*t3475
  t3501 = t3060*t3484
  t3502 = -0.103955395616*t2966*t3492
  t3503 = t3500 + t3501 + t3502
  t3505 = t3070*t3475
  t3506 = 0.994522*t2969*t3484
  t3507 = 0.104528*t2969*t3492
  t3508 = t3505 + t3506 + t3507
  t3520 = t2710*t2673*t2722
  t3521 = -1.0 * t2729*t2730
  t3522 = t3520 + t3521
  t3525 = t2705*t3522
  t3526 = t3525 + t2793
  t3528 = t2685*t3522
  t3529 = -1.0 * t2705*t2756
  t3530 = t3528 + t3529
  t3532 = -0.103955395616*t2614*t3526
  t3533 = t2918*t3530
  t3534 = t3532 + t3533
  t3536 = t2878*t3526
  t3537 = -0.103955395616*t2614*t3530
  t3538 = t3536 + t3537
  t3540 = 0.994522*t2654*t3526
  t3541 = 0.104528*t2654*t3530
  t3544 = t3540 + t3541
  t3547 = t2988*t3534
  t3548 = t2983*t3538
  t3549 = t2999*t3544
  t3552 = t3547 + t3548 + t3549
  t3554 = t3017*t3534
  t3555 = t3014*t3538
  t3556 = t3022*t3544
  t3557 = t3554 + t3555 + t3556
  t3559 = t3038*t3534
  t3560 = t3031*t3538
  t3561 = t3041*t3544
  t3562 = t3559 + t3560 + t3561
  t3564 = -0.104528*t2969*t3552
  t3565 = -0.103955395616*t2966*t3557
  t3566 = t3052*t3562
  t3568 = t3564 + t3565 + t3566
  t3570 = -0.994522*t2969*t3552
  t3571 = t3060*t3557
  t3572 = -0.103955395616*t2966*t3562
  t3573 = t3570 + t3571 + t3572
  t3575 = t3070*t3552
  t3576 = 0.994522*t2969*t3557
  t3577 = 0.104528*t2969*t3562
  t3580 = t3575 + t3576 + t3577
  t3594 = t2570*t2710*t2705
  t3595 = -1.0 * t2685*t2570*t2730
  t3596 = t3594 + t3595
  t3598 = t2685*t2570*t2710
  t3599 = t2570*t2705*t2730
  t3600 = t3598 + t3599
  t3602 = -0.103955395616*t2614*t3596
  t3603 = t2918*t3600
  t3604 = t3602 + t3603
  t3607 = t2878*t3596
  t3609 = -0.103955395616*t2614*t3600
  t3610 = t3607 + t3609
  t3615 = 0.994522*t2654*t3596
  t3616 = 0.104528*t2654*t3600
  t3617 = t3615 + t3616
  t3619 = t2988*t3604
  t3620 = t2983*t3610
  t3622 = t2999*t3617
  t3623 = t3619 + t3620 + t3622
  t3626 = t3017*t3604
  t3627 = t3014*t3610
  t3628 = t3022*t3617
  t3629 = t3626 + t3627 + t3628
  t3631 = t3038*t3604
  t3632 = t3031*t3610
  t3633 = t3041*t3617
  t3634 = t3631 + t3632 + t3633
  t3636 = -0.104528*t2969*t3623
  t3637 = -0.103955395616*t2966*t3629
  t3638 = t3052*t3634
  t3639 = t3636 + t3637 + t3638
  t3641 = -0.994522*t2969*t3623
  t3642 = t3060*t3629
  t3643 = -0.103955395616*t2966*t3634
  t3644 = t3641 + t3642 + t3643
  t3646 = t3070*t3623
  t3647 = 0.994522*t2969*t3629
  t3648 = 0.104528*t2969*t3634
  t3649 = t3646 + t3647 + t3648
  t3669 = -1.0 * t2685*t3095
  t3670 = -1.0 * t2705*t3101
  t3671 = t3669 + t3670
  t3674 = t2918*t3671
  t3675 = t3115 + t3674
  t3679 = t2878*t3106
  t3680 = -0.103955395616*t2614*t3671
  t3681 = t3679 + t3680
  t3683 = 0.994522*t2654*t3106
  t3684 = 0.104528*t2654*t3671
  t3685 = t3683 + t3684
  t3687 = t2988*t3675
  t3688 = t2983*t3681
  t3689 = t2999*t3685
  t3690 = t3687 + t3688 + t3689
  t3692 = t3017*t3675
  t3693 = t3014*t3681
  t3695 = t3022*t3685
  t3696 = t3692 + t3693 + t3695
  t3698 = t3038*t3675
  t3699 = t3031*t3681
  t3700 = t3041*t3685
  t3701 = t3698 + t3699 + t3700
  t3703 = -0.104528*t2969*t3690
  t3704 = -0.103955395616*t2966*t3696
  t3705 = t3052*t3701
  t3706 = t3703 + t3704 + t3705
  t3708 = -0.994522*t2969*t3690
  t3709 = t3060*t3696
  t3710 = -0.103955395616*t2966*t3701
  t3711 = t3708 + t3709 + t3710
  t3713 = t3070*t3690
  t3714 = 0.994522*t2969*t3696
  t3715 = 0.104528*t2969*t3701
  t3716 = t3713 + t3714 + t3715
  t3661 = -0.12*t2685
  t3662 = 0.4*t2705
  t3663 = t3661 + t3662
  t3665 = -0.4*t2685
  t3666 = t3665 + t2706
  t3729 = t2729*t2710
  t3730 = t2673*t2722*t2730
  t3731 = t3729 + t3730
  t3733 = -1.0 * t2705*t3522
  t3734 = t2685*t3731
  t3735 = t3733 + t3734
  t3738 = -1.0 * t2685*t3522
  t3739 = -1.0 * t2705*t3731
  t3740 = t3738 + t3739
  t3742 = -0.103955395616*t2614*t3735
  t3743 = t2918*t3740
  t3744 = t3742 + t3743
  t3746 = t2878*t3735
  t3747 = -0.103955395616*t2614*t3740
  t3748 = t3746 + t3747
  t3750 = 0.994522*t2654*t3735
  t3751 = 0.104528*t2654*t3740
  t3752 = t3750 + t3751
  t3754 = t2988*t3744
  t3755 = t2983*t3748
  t3756 = t2999*t3752
  t3757 = t3754 + t3755 + t3756
  t3759 = t3017*t3744
  t3760 = t3014*t3748
  t3761 = t3022*t3752
  t3762 = t3759 + t3760 + t3761
  t3764 = t3038*t3744
  t3765 = t3031*t3748
  t3766 = t3041*t3752
  t3767 = t3764 + t3765 + t3766
  t3769 = -0.104528*t2969*t3757
  t3770 = -0.103955395616*t2966*t3762
  t3771 = t3052*t3767
  t3772 = t3769 + t3770 + t3771
  t3774 = -0.994522*t2969*t3757
  t3775 = t3060*t3762
  t3776 = -0.103955395616*t2966*t3767
  t3777 = t3774 + t3775 + t3776
  t3779 = t3070*t3757
  t3780 = 0.994522*t2969*t3762
  t3781 = 0.104528*t2969*t3767
  t3782 = t3779 + t3780 + t3781
  t3796 = -1.0 * t2570*t2710*t2705
  t3797 = t2685*t2570*t2730
  t3798 = t3796 + t3797
  t3800 = -1.0 * t2685*t2570*t2710
  t3801 = -1.0 * t2570*t2705*t2730
  t3802 = t3800 + t3801
  t3804 = -0.103955395616*t2614*t3798
  t3805 = t2918*t3802
  t3806 = t3804 + t3805
  t3808 = t2878*t3798
  t3809 = -0.103955395616*t2614*t3802
  t3810 = t3808 + t3809
  t3812 = 0.994522*t2654*t3798
  t3813 = 0.104528*t2654*t3802
  t3814 = t3812 + t3813
  t3816 = t2988*t3806
  t3818 = t2983*t3810
  t3819 = t2999*t3814
  t3820 = t3816 + t3818 + t3819
  t3822 = t3017*t3806
  t3823 = t3014*t3810
  t3824 = t3022*t3814
  t3825 = t3822 + t3823 + t3824
  t3827 = t3038*t3806
  t3828 = t3031*t3810
  t3829 = t3041*t3814
  t3830 = t3827 + t3828 + t3829
  t3832 = -0.104528*t2969*t3820
  t3833 = -0.103955395616*t2966*t3825
  t3834 = t3052*t3830
  t3835 = t3832 + t3833 + t3834
  t3837 = -0.994522*t2969*t3820
  t3838 = t3060*t3825
  t3839 = -0.103955395616*t2966*t3830
  t3840 = t3837 + t3838 + t3839
  t3842 = t3070*t3820
  t3843 = 0.994522*t2969*t3825
  t3844 = 0.104528*t2969*t3830
  t3845 = t3842 + t3843 + t3844
  t3869 = -1.0000001112680001*t2729*t2570*t2654
  t3870 = 0.104528*t2604*t3106
  t3871 = 0.994522*t2604*t3112
  t3872 = t3869 + t3870 + t3871
  t3874 = -0.994522*t2604*t2729*t2570
  t3875 = -0.103955395616*t2654*t3106
  t3876 = -0.9890740084840001*t2654*t3112
  t3877 = t3874 + t3875 + t3876
  t3879 = -0.104528*t2604*t2729*t2570
  t3880 = -0.010926102783999999*t2654*t3106
  t3881 = -0.103955395616*t2654*t3112
  t3882 = t3879 + t3880 + t3881
  t3884 = t3041*t3872
  t3885 = t3031*t3877
  t3886 = t3038*t3882
  t3887 = t3884 + t3885 + t3886
  t3889 = t3022*t3872
  t3890 = t3014*t3877
  t3891 = t3017*t3882
  t3892 = t3889 + t3890 + t3891
  t3894 = t2999*t3872
  t3895 = t2983*t3877
  t3896 = t2988*t3882
  t3897 = t3894 + t3895 + t3896
  t3904 = -0.103955395616*t2966*t3887
  t3905 = t3060*t3892
  t3906 = -0.994522*t2969*t3897
  t3907 = t3904 + t3905 + t3906
  t3909 = t3052*t3887
  t3910 = -0.103955395616*t2966*t3892
  t3911 = -0.104528*t2969*t3897
  t3912 = t3909 + t3910 + t3911
  t3899 = 0.104528*t2969*t3887
  t3900 = 0.994522*t2969*t3892
  t3901 = t3070*t3897
  t3902 = t3899 + t3900 + t3901
  t3857 = 0.3852490428658858*t2604
  t3858 = -0.056500534356700764*t2654
  t3859 = t3857 + t3858
  t3861 = 0.0059058871981009595*t2604
  t3862 = 0.0402693119526853*t2654
  t3863 = 1.1345904784751044e-7 + t3861 + t3862
  t3865 = 0.05619101817723254*t2604
  t3866 = 0.3831386486090665*t2654
  t3867 = -1.1924972351948546e-8 + t3865 + t3866
  t3926 = t2705*t3731
  t3927 = t3528 + t3926
  t3929 = -1.0000001112680001*t2570*t2654*t2673
  t3930 = 0.104528*t2604*t3735
  t3931 = 0.994522*t2604*t3927
  t3932 = t3929 + t3930 + t3931
  t3934 = -0.994522*t2604*t2570*t2673
  t3935 = -0.103955395616*t2654*t3735
  t3936 = -0.9890740084840001*t2654*t3927
  t3937 = t3934 + t3935 + t3936
  t3940 = -0.104528*t2604*t2570*t2673
  t3941 = -0.010926102783999999*t2654*t3735
  t3942 = -0.103955395616*t2654*t3927
  t3943 = t3940 + t3941 + t3942
  t3945 = t3041*t3932
  t3946 = t3031*t3937
  t3947 = t3038*t3943
  t3948 = t3945 + t3946 + t3947
  t3950 = t3022*t3932
  t3951 = t3014*t3937
  t3952 = t3017*t3943
  t3953 = t3950 + t3951 + t3952
  t3955 = t2999*t3932
  t3956 = t2983*t3937
  t3957 = t2988*t3943
  t3958 = t3955 + t3956 + t3957
  t3965 = -0.103955395616*t2966*t3948
  t3966 = t3060*t3953
  t3967 = -0.994522*t2969*t3958
  t3968 = t3965 + t3966 + t3967
  t3970 = t3052*t3948
  t3971 = -0.103955395616*t2966*t3953
  t3972 = -0.104528*t2969*t3958
  t3973 = t3970 + t3971 + t3972
  t3960 = 0.104528*t2969*t3948
  t3961 = 0.994522*t2969*t3953
  t3962 = t3070*t3958
  t3963 = t3960 + t3961 + t3962
  t3988 = 1.0000001112680001*t2654*t2722
  t3989 = 0.104528*t2604*t3798
  t3990 = 0.994522*t2604*t3600
  t3991 = t3988 + t3989 + t3990
  t3993 = 0.994522*t2604*t2722
  t3994 = -0.103955395616*t2654*t3798
  t3995 = -0.9890740084840001*t2654*t3600
  t3996 = t3993 + t3994 + t3995
  t3998 = 0.104528*t2604*t2722
  t3999 = -0.010926102783999999*t2654*t3798
  t4000 = -0.103955395616*t2654*t3600
  t4001 = t3998 + t3999 + t4000
  t4003 = t3041*t3991
  t4004 = t3031*t3996
  t4005 = t3038*t4001
  t4006 = t4003 + t4004 + t4005
  t4008 = t3022*t3991
  t4009 = t3014*t3996
  t4010 = t3017*t4001
  t4011 = t4008 + t4009 + t4010
  t4013 = t2999*t3991
  t4014 = t2983*t3996
  t4015 = t2988*t4001
  t4016 = t4013 + t4014 + t4015
  t4023 = -0.103955395616*t2966*t4006
  t4024 = t3060*t4011
  t4025 = -0.994522*t2969*t4016
  t4026 = t4023 + t4024 + t4025
  t4028 = t3052*t4006
  t4029 = -0.103955395616*t2966*t4011
  t4030 = -0.104528*t2969*t4016
  t4031 = t4028 + t4029 + t4030
  t4018 = 0.104528*t2969*t4006
  t4019 = 0.994522*t2969*t4011
  t4020 = t3070*t4016
  t4021 = t4018 + t4019 + t4020
  t4044 = -0.051978134642000004*t2849
  t4058 = 0.05226439969100001*t2849
  t4048 = 0.49726168403800003*t2849
  t4043 = -0.707107*t2839
  t4047 = -0.073913*t2839
  t4054 = 0.707107*t2839
  t4075 = 0.051978134642000004*t2849
  t4057 = 0.703234*t2839
  t4068 = 0.073913*t2839
  t4078 = -0.49726168403800003*t2849
  t4065 = -0.703234*t2839
  t4086 = -0.05226439969100001*t2849
  t4074 = -0.5054634410180001*t2849*t3117
  t4076 = t4043 + t4075
  t4077 = t4076*t3127
  t4079 = t4047 + t4078
  t4080 = t4079*t3135
  t4081 = t4074 + t4077 + t4080
  t4083 = t4054 + t4075
  t4084 = t4083*t3117
  t4085 = -0.9945383682050002*t2849*t3127
  t4087 = t4057 + t4086
  t4088 = t4087*t3135
  t4089 = t4084 + t4085 + t4088
  t4091 = t4068 + t4078
  t4092 = t4091*t3117
  t4093 = t4065 + t4086
  t4094 = t4093*t3127
  t4095 = -0.500001190325*t2849*t3135
  t4096 = t4092 + t4094 + t4095
  t4103 = t3060*t4081
  t4104 = -0.103955395616*t2966*t4089
  t4105 = -0.994522*t2969*t4096
  t4106 = t4103 + t4104 + t4105
  t4108 = -0.103955395616*t2966*t4081
  t4109 = t3052*t4089
  t4110 = -0.104528*t2969*t4096
  t4111 = t4108 + t4109 + t4110
  t4098 = 0.994522*t2969*t4081
  t4099 = 0.104528*t2969*t4089
  t4100 = t3070*t4096
  t4101 = t4098 + t4099 + t4100
  t4045 = t4043 + t4044
  t4046 = -0.23105307644*t4045
  t4049 = t4047 + t4048
  t4050 = 0.164374659834*t4049
  t4051 = 0.0958179942122405*t2849
  t4052 = 4.0332087336819504e-7 + t4046 + t4050 + t4051
  t4055 = t4054 + t4044
  t4056 = 0.189564637987*t4055
  t4059 = t4057 + t4058
  t4060 = 0.164374659834*t4059
  t4061 = -0.22979114961138278*t2849
  t4063 = 4.239080549754904e-8 + t4056 + t4060 + t4061
  t4066 = t4065 + t4058
  t4067 = -0.23105307644*t4066
  t4069 = t4068 + t4048
  t4070 = 0.189564637987*t4069
  t4071 = 0.08218752557626696*t2849
  t4072 = -4.05542127947119e-7 + t4067 + t4070 + t4071
  t4123 = -0.994522*t2570*t2654*t2673
  t4124 = t2878*t3927
  t4125 = t4123 + t3742 + t4124
  t4127 = -0.104528*t2570*t2654*t2673
  t4128 = t2918*t3735
  t4129 = -0.103955395616*t2614*t3927
  t4130 = t4127 + t4128 + t4129
  t4132 = t2949*t2570*t2673
  t4133 = 0.104528*t2654*t3735
  t4134 = 0.994522*t2654*t3927
  t4135 = t4132 + t4133 + t4134
  t4137 = -0.5054634410180001*t2849*t4125
  t4138 = t4076*t4130
  t4139 = t4079*t4135
  t4140 = t4137 + t4138 + t4139
  t4142 = t4083*t4125
  t4143 = -0.9945383682050002*t2849*t4130
  t4144 = t4087*t4135
  t4145 = t4142 + t4143 + t4144
  t4147 = t4091*t4125
  t4148 = t4093*t4130
  t4149 = -0.500001190325*t2849*t4135
  t4150 = t4147 + t4148 + t4149
  t4157 = t3060*t4140
  t4158 = -0.103955395616*t2966*t4145
  t4159 = -0.994522*t2969*t4150
  t4160 = t4157 + t4158 + t4159
  t4162 = -0.103955395616*t2966*t4140
  t4163 = t3052*t4145
  t4164 = -0.104528*t2969*t4150
  t4165 = t4162 + t4163 + t4164
  t4152 = 0.994522*t2969*t4140
  t4153 = 0.104528*t2969*t4145
  t4154 = t3070*t4150
  t4155 = t4152 + t4153 + t4154
  t4177 = 0.994522*t2654*t2722
  t4178 = t2878*t3600
  t4179 = t4177 + t3804 + t4178
  t4181 = 0.104528*t2654*t2722
  t4182 = t2918*t3798
  t4183 = t4181 + t4182 + t3609
  t4185 = -1.0 * t2949*t2722
  t4186 = 0.104528*t2654*t3798
  t4187 = 0.994522*t2654*t3600
  t4188 = t4185 + t4186 + t4187
  t4190 = -0.5054634410180001*t2849*t4179
  t4191 = t4076*t4183
  t4192 = t4079*t4188
  t4193 = t4190 + t4191 + t4192
  t4195 = t4083*t4179
  t4196 = -0.9945383682050002*t2849*t4183
  t4197 = t4087*t4188
  t4198 = t4195 + t4196 + t4197
  t4200 = t4091*t4179
  t4201 = t4093*t4183
  t4202 = -0.500001190325*t2849*t4188
  t4203 = t4200 + t4201 + t4202
  t4210 = t3060*t4193
  t4211 = -0.103955395616*t2966*t4198
  t4212 = -0.994522*t2969*t4203
  t4213 = t4210 + t4211 + t4212
  t4215 = -0.103955395616*t2966*t4193
  t4216 = t3052*t4198
  t4217 = -0.104528*t2969*t4203
  t4218 = t4215 + t4216 + t4217
  t4205 = 0.994522*t2969*t4193
  t4206 = 0.104528*t2969*t4198
  t4207 = t3070*t4203
  t4208 = t4205 + t4206 + t4207
  t4247 = -0.994522*t2961*t3142
  t4248 = -0.9890740084840001*t2969*t3151
  t4249 = -0.103955395616*t2969*t3160
  t4250 = t4247 + t4248 + t4249
  t4252 = -0.104528*t2961*t3142
  t4253 = -0.103955395616*t2969*t3151
  t4254 = -0.010926102783999999*t2969*t3160
  t4255 = t4252 + t4253 + t4254
  t4242 = -1.0000001112680001*t2969*t3142
  t4243 = 0.994522*t2961*t3151
  t4244 = 0.104528*t2961*t3160
  t4245 = t4242 + t4243 + t4244
  t4230 = 0.13776101532839094*t2961
  t4231 = 0.19098732144477495*t2969
  t4232 = t4230 + t4231
  t4234 = -0.18994107176353728*t2961
  t4235 = 0.13700636048642204*t2969
  t4236 = 5.06291820800569e-8 + t4234 + t4235
  t4238 = -0.019963520514678434*t2961
  t4239 = 0.014399883410246048*t2969
  t4240 = -4.817066759205414e-7 + t4238 + t4239
  t4267 = t2983*t4125
  t4268 = t2988*t4130
  t4269 = t2999*t4135
  t4270 = t4267 + t4268 + t4269
  t4272 = t3014*t4125
  t4273 = t3017*t4130
  t4274 = t3022*t4135
  t4275 = t4272 + t4273 + t4274
  t4277 = t3031*t4125
  t4278 = t3038*t4130
  t4279 = t3041*t4135
  t4280 = t4277 + t4278 + t4279
  t4287 = -0.994522*t2961*t4270
  t4288 = -0.9890740084840001*t2969*t4275
  t4289 = -0.103955395616*t2969*t4280
  t4290 = t4287 + t4288 + t4289
  t4292 = -0.104528*t2961*t4270
  t4293 = -0.103955395616*t2969*t4275
  t4294 = -0.010926102783999999*t2969*t4280
  t4295 = t4292 + t4293 + t4294
  t4282 = -1.0000001112680001*t2969*t4270
  t4283 = 0.994522*t2961*t4275
  t4284 = 0.104528*t2961*t4280
  t4285 = t4282 + t4283 + t4284
  t4307 = t2983*t4179
  t4308 = t2988*t4183
  t4309 = t2999*t4188
  t4310 = t4307 + t4308 + t4309
  t4312 = t3014*t4179
  t4313 = t3017*t4183
  t4314 = t3022*t4188
  t4315 = t4312 + t4313 + t4314
  t4317 = t3031*t4179
  t4318 = t3038*t4183
  t4319 = t3041*t4188
  t4320 = t4317 + t4318 + t4319
  t4327 = -0.994522*t2961*t4310
  t4328 = -0.9890740084840001*t2969*t4315
  t4329 = -0.103955395616*t2969*t4320
  t4330 = t4327 + t4328 + t4329
  t4332 = -0.104528*t2961*t4310
  t4333 = -0.103955395616*t2969*t4315
  t4334 = -0.010926102783999999*t2969*t4320
  t4335 = t4332 + t4333 + t4334
  t4322 = -1.0000001112680001*t2969*t4310
  t4323 = 0.994522*t2961*t4315
  t4324 = 0.104528*t2961*t4320
  t4325 = t4322 + t4323 + t4324
  p_output1[0+1]=0
  p_output1[1+1]=0
  p_output1[2+1]=0
  p_output1[3+1]=0
  p_output1[4+1]=0
  p_output1[5+1]=0
  p_output1[6+1]=0
  p_output1[7+1]=0
  p_output1[8+1]=0
  p_output1[9+1]=-1.0 * t2570*t2667*t2673 + t2707*t2738 + t2749*t2756 + t2788*t2799 + t2804*t2823 + t2872*t2881 + t2909*t2925 + t2946*t2955 + t2975*t3003 + t3012*t3024 + t3030*t3044 - 0.272124*t3054 - 0.07912*(-0.994522*t3054 + 0.104528*t3065) + 0.167122*t3065 + 0.369*(0.040001*t3054 + 0.380588*t3065 + 0.92388*t3075) + 0.190987*t3075
  p_output1[10+1]=t2570*t2667*t2729 + t2707*t3095 + t2749*t3101 + t2788*t3106 + t2804*t3112 + t2872*t3117 + t2909*t3127 + t2946*t3135 + t2975*t3142 + t3012*t3151 + t3030*t3160 - 0.272124*t3169 - 0.07912*(-0.994522*t3169 + 0.104528*t3174) + 0.167122*t3174 + 0.369*(0.040001*t3169 + 0.380588*t3174 + 0.92388*t3180) + 0.190987*t3180
  p_output1[11+1]=0
  p_output1[12+1]=t2570*t2707*t2710*t2729 - 1.0 * t2667*t2722*t2729 + t2570*t2729*t2730*t2749 + t2788*t3202 + t2804*t3206 + t2872*t3212 + t2909*t3217 + t2946*t3227 + t2975*t3232 + t3012*t3237 + t3030*t3245 - 0.272124*t3250 - 0.07912*(-0.994522*t3250 + 0.104528*t3258) + 0.167122*t3258 + 0.369*(0.040001*t3250 + 0.380588*t3258 + 0.92388*t3266) + 0.190987*t3266
  p_output1[13+1]=t2570*t2673*t2707*t2710 - 1.0 * t2667*t2673*t2722 + t2570*t2673*t2730*t2749 + t2788*t3292 + t2804*t3298 + t2872*t3303 + t2909*t3308 + t2946*t3314 + t2975*t3322 + t3012*t3328 + t3030*t3334 - 0.272124*t3340 - 0.07912*(-0.994522*t3340 + 0.104528*t3345) + 0.167122*t3345 + 0.369*(0.040001*t3340 + 0.380588*t3345 + 0.92388*t3350) + 0.190987*t3350
  p_output1[14+1]=-1.0 * t2570*t2667 - 1.0 * t2707*t2710*t2722 - 1.0 * t2722*t2730*t2749 + t2788*t3372 + t2804*t3376 + t2872*t3381 + t2909*t3386 + t2946*t3393 + t2975*t3399 + t3012*t3404 + t3030*t3409 - 0.272124*t3419 - 0.07912*(-0.994522*t3419 + 0.104528*t3428) + 0.167122*t3428 + 0.369*(0.040001*t3419 + 0.380588*t3428 + 0.92388*t3433) + 0.190987*t3433
  p_output1[15+1]=t2749*t3095 + t2707*t3448 + t2804*t3452 + t2788*t3456 + t2909*t3461 + t2872*t3465 + t2946*t3470 + t2975*t3475 + t3012*t3484 + t3030*t3492 - 0.272124*t3498 - 0.07912*(-0.994522*t3498 + 0.104528*t3503) + 0.167122*t3503 + 0.369*(0.040001*t3498 + 0.380588*t3503 + 0.92388*t3508) + 0.190987*t3508
  p_output1[16+1]=t2707*t2756 + t2749*t3522 + t2804*t3526 + t2788*t3530 + t2909*t3534 + t2872*t3538 + t2946*t3544 + t2975*t3552 + t3012*t3557 + t3030*t3562 - 0.272124*t3568 - 0.07912*(-0.994522*t3568 + 0.104528*t3573) + 0.167122*t3573 + 0.369*(0.040001*t3568 + 0.380588*t3573 + 0.92388*t3580) + 0.190987*t3580
  p_output1[17+1]=-1.0 * t2570*t2707*t2730 + t2570*t2710*t2749 + t2804*t3596 + t2788*t3600 + t2909*t3604 + t2872*t3610 + t2946*t3617 + t2975*t3623 + t3012*t3629 + t3030*t3634 - 0.272124*t3639 - 0.07912*(-0.994522*t3639 + 0.104528*t3644) + 0.167122*t3644 + 0.369*(0.040001*t3639 + 0.380588*t3644 + 0.92388*t3649) + 0.190987*t3649
  p_output1[18+1]=0
  p_output1[19+1]=0
  p_output1[20+1]=0
  p_output1[21+1]=0
  p_output1[22+1]=0
  p_output1[23+1]=0
  p_output1[24+1]=0
  p_output1[25+1]=0
  p_output1[26+1]=0
  p_output1[27+1]=0
  p_output1[28+1]=0
  p_output1[29+1]=0
  p_output1[30+1]=0
  p_output1[31+1]=0
  p_output1[32+1]=0
  p_output1[33+1]=0
  p_output1[34+1]=0
  p_output1[35+1]=0
  p_output1[36+1]=0
  p_output1[37+1]=0
  p_output1[38+1]=0
  p_output1[39+1]=0
  p_output1[40+1]=0
  p_output1[41+1]=0
  p_output1[42+1]=0
  p_output1[43+1]=0
  p_output1[44+1]=0
  p_output1[45+1]=0
  p_output1[46+1]=0
  p_output1[47+1]=0
  p_output1[48+1]=0
  p_output1[49+1]=0
  p_output1[50+1]=0
  p_output1[51+1]=0
  p_output1[52+1]=0
  p_output1[53+1]=0
  p_output1[54+1]=0
  p_output1[55+1]=0
  p_output1[56+1]=0
  p_output1[57+1]=0
  p_output1[58+1]=0
  p_output1[59+1]=0
  p_output1[60+1]=0
  p_output1[61+1]=0
  p_output1[62+1]=0
  p_output1[63+1]=0
  p_output1[64+1]=0
  p_output1[65+1]=0
  p_output1[66+1]=0
  p_output1[67+1]=0
  p_output1[68+1]=0
  p_output1[69+1]=0
  p_output1[70+1]=0
  p_output1[71+1]=0
  p_output1[72+1]=0
  p_output1[73+1]=0
  p_output1[74+1]=0
  p_output1[75+1]=0
  p_output1[76+1]=0
  p_output1[77+1]=0
  p_output1[78+1]=t2804*t3106 + t3095*t3663 + t3101*t3666 + t2788*t3671 + t2909*t3675 + t2872*t3681 + t2946*t3685 + t2975*t3690 + t3012*t3696 + t3030*t3701 - 0.272124*t3706 - 0.07912*(-0.994522*t3706 + 0.104528*t3711) + 0.167122*t3711 + 0.369*(0.040001*t3706 + 0.380588*t3711 + 0.92388*t3716) + 0.190987*t3716
  p_output1[79+1]=t3522*t3663 + t3666*t3731 + t2804*t3735 + t2788*t3740 + t2909*t3744 + t2872*t3748 + t2946*t3752 + t2975*t3757 + t3012*t3762 + t3030*t3767 - 0.272124*t3772 - 0.07912*(-0.994522*t3772 + 0.104528*t3777) + 0.167122*t3777 + 0.369*(0.040001*t3772 + 0.380588*t3777 + 0.92388*t3782) + 0.190987*t3782
  p_output1[80+1]=t2570*t2710*t3663 + t2570*t2730*t3666 + t2804*t3798 + t2788*t3802 + t2909*t3806 + t2872*t3810 + t2946*t3814 + t2975*t3820 + t3012*t3825 + t3030*t3830 - 0.272124*t3835 - 0.07912*(-0.994522*t3835 + 0.104528*t3840) + 0.167122*t3840 + 0.369*(0.040001*t3835 + 0.380588*t3840 + 0.92388*t3845) + 0.190987*t3845
  p_output1[81+1]=t2570*t2729*t3859 + t3106*t3863 + t3112*t3867 + t2946*t3872 + t2872*t3877 + t2909*t3882 + t3030*t3887 + t3012*t3892 + t2975*t3897 + 0.190987*t3902 + 0.167122*t3907 - 0.07912*(0.104528*t3907 - 0.994522*t3912) + 0.369*(0.92388*t3902 + 0.380588*t3907 + 0.040001*t3912) - 0.272124*t3912
  p_output1[82+1]=t2570*t2673*t3859 + t3735*t3863 + t3867*t3927 + t2946*t3932 + t2872*t3937 + t2909*t3943 + t3030*t3948 + t3012*t3953 + t2975*t3958 + 0.190987*t3963 + 0.167122*t3968 - 0.07912*(0.104528*t3968 - 0.994522*t3973) + 0.369*(0.92388*t3963 + 0.380588*t3968 + 0.040001*t3973) - 0.272124*t3973
  p_output1[83+1]=-1.0 * t2722*t3859 + t3798*t3863 + t3600*t3867 + t2946*t3991 + t2872*t3996 + t2909*t4001 + t3030*t4006 + t3012*t4011 + t2975*t4016 + 0.190987*t4021 + 0.167122*t4026 - 0.07912*(0.104528*t4026 - 0.994522*t4031) + 0.369*(0.92388*t4021 + 0.380588*t4026 + 0.040001*t4031) - 0.272124*t4031
  p_output1[84+1]=t3117*t4052 + t3127*t4063 + t3135*t4072 + t3012*t4081 + t3030*t4089 + t2975*t4096 + 0.190987*t4101 + 0.167122*t4106 - 0.07912*(0.104528*t4106 - 0.994522*t4111) + 0.369*(0.92388*t4101 + 0.380588*t4106 + 0.040001*t4111) - 0.272124*t4111
  p_output1[85+1]=t4052*t4125 + t4063*t4130 + t4072*t4135 + t3012*t4140 + t3030*t4145 + t2975*t4150 + 0.190987*t4155 + 0.167122*t4160 - 0.07912*(0.104528*t4160 - 0.994522*t4165) + 0.369*(0.92388*t4155 + 0.380588*t4160 + 0.040001*t4165) - 0.272124*t4165
  p_output1[86+1]=t4052*t4179 + t4063*t4183 + t4072*t4188 + t3012*t4193 + t3030*t4198 + t2975*t4203 + 0.190987*t4208 + 0.167122*t4213 - 0.07912*(0.104528*t4213 - 0.994522*t4218) + 0.369*(0.92388*t4208 + 0.380588*t4213 + 0.040001*t4218) - 0.272124*t4218
  p_output1[87+1]=t3142*t4232 + t3151*t4236 + t3160*t4240 + 0.190987*t4245 + 0.167122*t4250 - 0.07912*(0.104528*t4250 - 0.994522*t4255) + 0.369*(0.92388*t4245 + 0.380588*t4250 + 0.040001*t4255) - 0.272124*t4255
  p_output1[88+1]=t4232*t4270 + t4236*t4275 + t4240*t4280 + 0.190987*t4285 + 0.167122*t4290 - 0.07912*(0.104528*t4290 - 0.994522*t4295) + 0.369*(0.92388*t4285 + 0.380588*t4290 + 0.040001*t4295) - 0.272124*t4295
  p_output1[89+1]=t4232*t4310 + t4236*t4315 + t4240*t4320 + 0.190987*t4325 + 0.167122*t4330 - 0.07912*(0.104528*t4330 - 0.994522*t4335) + 0.369*(0.92388*t4325 + 0.380588*t4330 + 0.040001*t4335) - 0.272124*t4335 
  
end



function Jp_right_hand_wrt_base(θ::AbstractVector{T}) where T
  right_hand = zeros(T, 3, 30)
  Jp_right_hand_wrt_base_helper!(right_hand, θ)
  return right_hand
end
