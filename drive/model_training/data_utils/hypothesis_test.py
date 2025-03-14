
from scipy import mannwhitneyu,ttest_ind
import numpy as np 
METRICS = ""


def compute_significance(values, reference, direction='greater'):
    
    return_dic = {metric: None for metric in METRICS}
    metric_significantly_different = []
    reference_distribution = values[reference]
    for metric in METRICS:
        if not metric == reference:
            value_coverage = values[metric]
            
            if len(value_coverage) < 20 or len(reference_distribution) < 20:
                raise ValueError("Each sample must have more than 20 observations for valid Mann-Whitney U results.")
            if not (np.issubdtype(np.array(value_coverage).dtype, np.number) and np.issubdtype(np.array(reference_distribution).dtype, np.number)):
                raise ValueError("Samples must be numeric.")
            
            # Manwithney u compare les deux distributions la forme
            p_value = mannwhitneyu(value_coverage, reference_distribution, alternative='two-sided').pvalue
            if p_value < 0.05:
                metric_significantly_different.append(metric)
            else:
                print(f"\033[93m{metric} is not different\033[0m")
                return_dic[metric] = r'$\equiv$'
    
    for metric_significant in metric_significantly_different:
        value_coverage = values[metric_significant]
        p_value_direction = ttest_ind(value_coverage, reference_distribution, equal_var=False, alternative=direction).pvalue
        # print(f"P-value ttest metric {metric_significant}: {p_value_direction}")
        # print(f"To be {direction} than {reference}, p-value must be less than {0.05/len(metric_significantly_different)}")
        if p_value_direction < 0.05/len(metric_significantly_different):
            print(f"\033[92mMetric {metric_significant} is significantly different ({direction}).\033[0m")
            return_dic[metric_significant] = r'\textcolor{ForestGreen}{$\blacktriangle$}'
        else:
            print(f"\033[91mMetric {metric_significant} is not significantly different ({'less' if direction == 'greater' else 'greater'}).\033[0m")
            return_dic[metric_significant] = r'\textcolor{Mahogany}{$\blacktriangledown$}'
            